//
//  Copyright (c) 2021-2022 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of kksdcmd.
//
//  kksdcmd is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  kksdcmd is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with kksdcmd. If not, see <http://www.gnu.org/licenses/>.
//


#include "application.hpp"

#include "macaddress.hpp"
#include "modbus.hpp"
#include "utils.hpp"
#include "jsonobject.hpp"
#include "analogio.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "spi.hpp"
#include "coreregmodel.hpp"

#if ENABLE_UBUS
  #include "ubus.hpp"
#endif

#include <stdio.h>
#include <math.h>

#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)
#define DEFAULT_MODBUS_IP_PORT 502 // standard modbus port
#define DEFAULT_POLL_INTERVAL_MS 500 // default proxy polling interval (SPI and modbus)

#define MAINSCRIPT_DEFAULT_FILE_NAME "mainscript.txt"

using namespace p44;
using namespace P44Script;


#if ENABLE_UBUS
static const struct blobmsg_policy logapi_policy[] = {
  { .name = "level", .type = BLOBMSG_TYPE_INT8 },
  { .name = "deltastamps", .type = BLOBMSG_TYPE_BOOL },
  { .name = NULL, .type = BLOBMSG_TYPE_INT32 },
};

static const struct blobmsg_policy kksmbcapi_policy[] = {
  { .name = "method", .type = BLOBMSG_TYPE_STRING },
  { .name = NULL, .type = BLOBMSG_TYPE_UNSPEC },
};
#endif // ENABLE_UBUS


static JsonObjectPtr makeResponse(JsonObjectPtr aResult, ErrorPtr aErr)
{
  JsonObjectPtr response = JsonObject::newObj();
  if (Error::notOK(aErr)) {
    response->add("errordomain", JsonObject::newString(aErr->domain()));
    response->add("error", JsonObject::newInt32((int32_t)aErr->getErrorCode()));
    response->add("errorname", JsonObject::newString(aErr->errorCodeText().c_str()));
    response->add("errormessage", JsonObject::newString(aErr->getErrorMessage()));
    if (aResult) response->add("result", aResult); // add non-empty result even in error case (not a common case)
  }
  else {
    response->add("result", aResult); // including empty result
  }
  return response;
}



#if ENABLE_P44SCRIPT && ENABLE_UBUS

// MARK: - ApiRequestObj

class ApiRequestObj : public ObjectValue
{
  typedef ObjectValue inherited;

  EventSource* mEventSource;
  UbusRequestPtr mUbusRequest;

public:
  ApiRequestObj(UbusRequestPtr aUbusRequest, EventSource* aApiEventSource) :
    inherited(aUbusRequest ? aUbusRequest->msg() : JsonObjectPtr()),
    mUbusRequest(aUbusRequest),
    mEventSource(aApiEventSource)
  {
  }

  void sendResponse(JsonObjectPtr aResponse, ErrorPtr aError)
  {
    if (mUbusRequest) mUbusRequest->sendResponse(makeResponse(aResponse, aError), UBUS_STATUS_OK);
    mUbusRequest.reset(); // done now
  }

  virtual string getAnnotation() const P44_OVERRIDE
  {
    return "API request";
  }

  virtual TypeInfo getTypeInfo() const P44_OVERRIDE
  {
    return inherited::getTypeInfo()|oneshot|keeporiginal; // returns the request only once, must keep the original
  }

  virtual bool isEventSource() const P44_OVERRIDE { return true; };
  
  virtual void registerForFilteredEvents(EventSink* aEventSink, intptr_t aRegId = 0) P44_OVERRIDE
  {
    mEventSource->registerForEvents(aEventSink, aRegId);
  }

  virtual const ScriptObjPtr memberByName(const string aName, TypeInfo aMemberAccessFlags = none) const P44_OVERRIDE;

};

// answer([answer value])        answer the request
FUNC_ARG_DEFS(answer, { anyvalid|optionalarg } );
static void answer_func(BuiltinFunctionContextPtr f)
{
  ApiRequestObj* reqObj = dynamic_cast<ApiRequestObj *>(f->thisObj().get());
  if (f->arg(0)->isErr()) {
    reqObj->sendResponse(JsonObjectPtr(), f->arg(0)->errorValue());
  }
  else {
    reqObj->sendResponse(f->arg(0)->jsonValue(), ErrorPtr());
  }
  f->finish();
}

static const BuiltinMemberDescriptor answer_desc =
  FUNC_DEF_W_ARG(answer, executable|anyvalid);


const ScriptObjPtr ApiRequestObj::memberByName(const string aName, TypeInfo aMemberAccessFlags) const
{
  ScriptObjPtr val;
  if (uequals(aName, "answer")) {
    val = new BuiltinFunctionObj(&answer_desc, const_cast<ApiRequestObj*>(this), nullptr);
  }
  else {
    val = inherited::memberByName(aName, aMemberAccessFlags);
  }
  return val;
}

class ScriptApiLookup;

static ScriptApiLookup* gScriptApiLookupP; // FIXME: ugly static pointer

// webrequest()        return latest unprocessed script (web) api request
static void webrequest_func(BuiltinFunctionContextPtr f);

static const BuiltinMemberDescriptor scriptApiGlobals[] = {
  FUNC_DEF_NOARG(webrequest, executable|objectvalue|null),
  { NULL } // terminator
};

/// represents the global objects related to p44features
class ScriptApiLookup : public BuiltInMemberLookup, public EventSource
{
  typedef BuiltInMemberLookup inherited;
  friend class KksDcmD;

  UbusRequestPtr mPendingScriptApiRequest; ///< pending script API request

public:
  ScriptApiLookup() : inherited(scriptApiGlobals) {};

  UbusRequestPtr pendingRequest()
  {
    UbusRequestPtr r = mPendingScriptApiRequest;
    mPendingScriptApiRequest.reset();
    return r;
  }

};


static void webrequest_func(BuiltinFunctionContextPtr f)
{
  // return latest unprocessed API request
  f->finish(new ApiRequestObj(gScriptApiLookupP->pendingRequest(), gScriptApiLookupP));
}

#endif // ENABLE_P44SCRIPT && ENABLE_UBUS



// MARK: - KksDcmD

class KksDcmD;

/// global script function lookup for this app
class KksDcmDLookup : public BuiltInMemberLookup
{
  typedef BuiltInMemberLookup inherited;
public:
  KksDcmD &mKksdcmd;
  KksDcmDLookup(KksDcmD &aKksdcmd);
};


class KksDcmD : public CmdLineApp
{
  typedef CmdLineApp inherited;

  #if ENABLE_UBUS
  // ubus API
  UbusServerPtr mUbusApiServer; ///< ubus API for openwrt web interface
  #endif // ENABLE_UBUS

  typedef std::vector<CoreRegModelPtr> CoreRegModelVector;

  CoreRegModelVector mCoreRegModels; ///< the core register models (local SPI and proxies)

  MLMicroSeconds mPollInterval = Never;
  MLTicket mPollTimer;

  // app
  bool mActive;

  #if ENABLE_P44SCRIPT
  // scripting
  string mMainScriptFn; ///< filename for the main script
  ScriptHost mMainScript;
  ScriptMainContextPtr mScriptMainContext;
  #if ENABLE_UBUS
  ScriptApiLookup mScriptApiLookup; ///< lookup and event source for script API
  #endif // ENABLE_UBUS
  #endif // ENABLE_P44SCRIPT

public:

  KksDcmD() :
    mMainScript(sourcecode+regular, "main") // only init script may have declarations
  {
    mActive = true;
    // let all scripts run in the same context

    #if ENABLE_P44SCRIPT
    mScriptMainContext = mMainScript.domain()->newContext();
    mMainScript.setSharedMainContext(mScriptMainContext);
    #if ENABLE_UBUS
    gScriptApiLookupP = &mScriptApiLookup; // FIXME: ugly static pointer
    #endif
    #endif // ENABLE_P44SCRIPT

  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
    "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      #if ENABLE_P44SCRIPT
      { 0  , "mainscript",    true,  "p44scriptfile;the main script to run after startup" },
      #endif
      #if ENABLE_UBUS
      { 0  , "ubusapi",       false, "enable ubus API" },
      #endif
      { 0  , "modbus",        true,  "ip:port;TCP address (0.0.0.0 for server) port to listen for modbus connections, default=none" },
      { 0  , "corespi",       true,  "busno*10+CSno;SPI bus and CS number to use, default=no SPI" },
      { 0  , "proxybaseip",   true,  "base ip;IP of first proxy (modbus slave), default=no proxies" },
      { 0  , "proxyport",     true,  "port;port number for modbus proxy connections, default=" STRINGIZE(DEFAULT_MODBUS_IP_PORT) },
      { 0  , "numproxies",    true,  "num;number of proxies (with consecutive IP addresses), default=1" },
      { 0  , "pollinterval",  true,  "milliseconds;refresh interval for locally cached registers from SPI and proxies, 0=Never, default=" STRINGIZE(DEFAULT_POLL_INTERVAL_MS) },
      CMDLINE_APPLICATION_PATHOPTIONS,
      DAEMON_APPLICATION_LOGOPTIONS,
      CMDLINE_APPLICATION_STDOPTIONS,
      { 0, NULL } // list terminator
    };

    // parse the command line, exits when syntax errors occur
    setCommandDescriptors(usageText, options);
    parseCommandLine(argc, argv);
    processStandardLogOptions(true); // daemon defaults

    if (numOptions()<1) {
      // show usage
      showUsage();
      terminateApp(EXIT_SUCCESS);
    }

    #if ENABLE_UBUS
    // Prepare ubus API
    if (getOption("ubusapi")) {
      initUbusApi();
    }
    #endif // ENABLE_UBUS

    // app now ready to run
    return run();
  }



  // MARK: - ubus API

  #if ENABLE_UBUS

  #define MAX_REG 64

  void initUbusApi()
  {
    mUbusApiServer = UbusServerPtr(new UbusServer());
    UbusObjectPtr u = new UbusObject("kksdcmd", boost::bind(&KksDcmD::ubusApiRequestHandler, this, _1));
    u->addMethod("log", logapi_policy);
    u->addMethod("api", kksmbcapi_policy);
    u->addMethod("quit");
    u->addMethod("version");
    mUbusApiServer->registerObject(u);
  }

  #if ENABLE_P44SCRIPT
  void scriptExecHandler(UbusRequestPtr aUbusRequest, ScriptObjPtr aResult)
  {
    JsonObjectPtr ans = JsonObject::newObj();
    if (aResult) {
      if (aResult->isErr()) {
        ans->add("error", ans->newString(aResult->errorValue()->text()));
      }
      else {
        ans->add("result", aResult->jsonValue());
      }
      ans->add("annotation", JsonObject::newString(aResult->getAnnotation()));
      SourceCursor *cursorP = aResult->cursor();
      if (cursorP) {
        ans->add("sourceline", JsonObject::newString(cursorP->linetext()));
        ans->add("at", JsonObject::newInt64(cursorP->textpos()));
        ans->add("line", JsonObject::newInt64(cursorP->lineno()));
        ans->add("char", JsonObject::newInt64(cursorP->charpos()));
      }
    }
    // a script exec response is always a "result" at the API level
    // (differentiating between error-type and non-error-type script results at a higher level)
    JsonObjectPtr msg = JsonObject::newObj();
    msg->add("result", ans);
    aUbusRequest->sendResponse(msg);
  }
  #endif // ENABLE_P44SCRIPT


  void ubusApiRequestHandler(UbusRequestPtr aUbusRequest)
  {
    if (aUbusRequest->method()=="log") {
      if (aUbusRequest->msg()) {
        JsonObjectPtr o;
        if (aUbusRequest->msg()->get("level", o)) {
          int oldLevel = LOGLEVEL;
          int newLevel = o->int32Value();
          SETLOGLEVEL(newLevel);
          LOG(newLevel, "\n\n========== changed log level from %d to %d ===============", oldLevel, newLevel);
        }
        if (aUbusRequest->msg()->get("deltastamps", o)) {
          SETDELTATIME(o->boolValue());
        }
      }
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aUbusRequest->method()=="quit") {
      LOG(LOG_WARNING, "terminated via UBUS quit method");
      terminateApp(1);
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aUbusRequest->method()=="version") {
      aUbusRequest->sendResponse(JsonObject::newString(Application::version()));
    }
    else if (aUbusRequest->method()=="api") {
      ErrorPtr err;
      JsonObjectPtr result;
      JsonObjectPtr o;
      if (aUbusRequest->msg()) {
        JsonObjectPtr subsys;
        // API for KKS-DCM core register web interface
        if (aUbusRequest->msg()->get("coreregs", subsys)) {
          if (!subsys->get("cmd", o)) {
            err = TextError::err("missing 'cmd' in 'coreregs'");
          }
          else {
            string cmd = o->stringValue();
            size_t generator = 0; // default to first (which is local SPI when enabled)
            if (subsys->get("generator", o)) {
              generator = o->int32Value();
              if (generator>=mCoreRegModels.size()) {
                err = TextError::err("'generator' out of range, must be 0..%zu", mCoreRegModels.size()-1);
              }
            }
            if (Error::isOK(err)) {
              if (cmd=="list") {
                // list current register model
                CoreRegModel::RegIndex from = 0;
                CoreRegModel::RegIndex to = mCoreRegModels[generator]->maxReg();
                if (subsys->get("index", o)) from = o->int32Value();
                if (subsys->get("count", o)) to = from+o->int32Value()-1;
                if (to<from) {
                  err = TextError::err("'count' must be >=1");
                }
                else {
                  if (subsys->get("refresh", o)) {
                    if (o->boolValue()) {
                      err = mCoreRegModels[generator]->updateRegisterCacheFromHardware(from, to);
                    }
                  }
                  result = mCoreRegModels[generator]->getRegisterInfos(from, to);
                }
              }
              else if (cmd=="read") {
                if (!subsys->get("index", o)) {
                  err = TextError::err("missing 'index' for 'read' command");
                }
                else {
                  CoreRegModel::RegIndex from = o->int32Value();
                  CoreRegModel::RegIndex to = from;
                  if (subsys->get("count", o)) to = from+o->int32Value()-1;
                  if (to<from) {
                    err = TextError::err("'count' must be >=1");
                  }
                  else {
                    if (subsys->get("refresh", o)) {
                      if (o->boolValue()) {
                        err = mCoreRegModels[generator]->updateRegisterCacheFromHardware(from, to);
                      }
                    }
                    if (Error::isOK(err)) {
                      if (to>from) {
                        // multiple registers, basically same as "list"
                        result = mCoreRegModels[generator]->getRegisterInfos(from, to);
                      }
                      else {
                        // single register, result is object of that single register, not wrapped in array
                        result = mCoreRegModels[generator]->getRegisterInfo(from);
                      }
                    }
                  }
                }
              }
              else if (cmd=="write") {
                // write Params:
                // - index: first register
                // - count: optional for writing or committing multiple registers
                // - commit: true or false -> commit value(s) to hardware
                // - value: single value or array of values (length of array implies count,
                //     if count is specified, array size must match
                //     if commit: true is passed, specifying no value(s) is allowed
                if (!subsys->get("index", o)) {
                  err = TextError::err("missing 'index' for 'write' command");
                }
                else {
                  // first register
                  CoreRegModel::RegIndex from = o->int32Value();
                  // check optional params
                  bool doCommit = true; // by default, commit
                  bool explicitCommit = false;
                  if (subsys->get("commit", o)) {
                    doCommit = o->boolValue();
                    explicitCommit = true;
                  }
                  CoreRegModel::RegIndex count = 1;
                  bool explicitCount = false;
                  if (subsys->get("count", o)) {
                    count = o->int32Value();
                    explicitCount = true;
                  }
                  // write new values
                  if (subsys->get("value", o)) {
                    // values to write
                    if (o->isType(json_type_array)) {
                      // multiple values
                      if (explicitCount && count!=o->arrayLength()) {
                        err = TextError::err("'count' does not match size of 'value' array");
                      }
                      else {
                        count = o->arrayLength();
                        // multiple values
                        for (int i=0; i<count; i++) {
                          err = mCoreRegModels[generator]->setRegisterValue(from+i, o->arrayGet(i));
                          if (Error::notOK(err)) break;
                        }
                      }
                    }
                    else {
                      // single value
                      if (explicitCount && count!=1) {
                        err = TextError::err("'count' must be 1 when 'value' is not an array");
                      }
                      else {
                        err = mCoreRegModels[generator]->setRegisterValue(from, o);
                      }
                    }
                  }
                  else {
                    // no values - might be commit-only
                    if (!explicitCount || !explicitCommit || !doCommit) {
                      err = TextError::err("with no 'value', 'count' and 'commit' must be set");
                    }
                  }
                  if (Error::isOK(err) && doCommit && count>0) {
                    // possibly commit values
                    err = mCoreRegModels[generator]->updateHardwareFromRegisterCache(from, from+count-1);
                  }
                }
              }
              else {
                err = TextError::err("unknown 'cmd'='%s' in 'coreregs'", cmd.c_str());
              }
            }
          }
        }
        #if ENABLE_P44SCRIPT
        // API for editing/starting/stopping mainscript via web interface
        if (aUbusRequest->msg()->get("mainscript", subsys)) {
          if (subsys->get("execcode", o)) {
            // direct execution of a script command line in the common main/initscript context
            ScriptHost src(sourcecode+regular+keepvars+concurrently+ephemeralSource, "execcode");
            src.setSource(o->stringValue());
            src.setSharedMainContext(mScriptMainContext);
            src.run(inherit, boost::bind(&KksDcmD::scriptExecHandler, this, aUbusRequest, _1));
            return;
          }
          bool newCode = false;
          bool execaction = false;
          if (subsys->get("stop", o) && o->boolValue()) {
            // stop
            mScriptMainContext->abort(stopall);
            execaction = true;
          }
          if (subsys->get("code", o)) {
            if (mMainScriptFn.empty()) {
              mMainScriptFn = MAINSCRIPT_DEFAULT_FILE_NAME;
            }
            // set new main script
            mScriptMainContext->abort(stopall);
            mMainScript.setSource(o->stringValue());
            // always: check it
            ScriptObjPtr res = mMainScript.syntaxcheck();
            ErrorPtr err;
            if (!res || !res->isErr()) {
              LOG(LOG_INFO, "Checked global main script: syntax OK");
              if (subsys->get("save", o) && o->boolValue()) {
                // save the script
                err = string_tofile(dataPath(mMainScriptFn), mMainScript.getSource());
              }
            }
            else {
              LOG(LOG_NOTICE, "Error in global main script: %s", res->errorValue()->text());
              scriptExecHandler(aUbusRequest, res);
              return;
            }
            newCode = true;
            // checked ok
          }
          if (subsys->get("run", o) && o->boolValue()) {
            // run the script
            LOG(LOG_NOTICE, "Re-starting global main script");
            mMainScript.run(stopall);
            execaction = true;
          }
          else if (!newCode && !execaction) {
            // return current mainscript code
            result = JsonObject::newObj();
            result->add("code", JsonObject::newString(mMainScript.getSource()));
          }
          else {
            // ok w/o result
            result = JsonObject::newObj();
          }
        }
        // API for interfacing Web App into p44script-implemented functionality (mainscript)
        if (aUbusRequest->msg()->get("scriptapi", subsys)) {
          // scripted parts of the (web) API
          if (!mScriptApiLookup.hasSinks()) {
            // no script API active
            err = WebError::webErr(500, "script API not active");
          }
          else {
            // pass as event to p44script to handle
            aUbusRequest->setMsg(subsys); // only pass the subsys
            mScriptApiLookup.mPendingScriptApiRequest = aUbusRequest;
            mScriptApiLookup.sendEvent(new ApiRequestObj(mScriptApiLookup.mPendingScriptApiRequest, &mScriptApiLookup));
          }
        }
        #endif // ENABLE_P44SCRIPT
      }
      else {
        err = TextError::err("missing command object");
      }
      aUbusRequest->sendResponse(makeResponse(result, err));
    }
    else {
      // no other methods implemented yet
      aUbusRequest->sendResponse(JsonObjectPtr(), UBUS_STATUS_INVALID_COMMAND);
    }
  }



  #endif // ENABLE_UBUS

  // MARK: - modbus access handler

  ErrorPtr modbusAccessHandler(int aAddress, bool aBit, bool aInput, bool aWrite)
  {
    ErrorPtr err;
    if (mCoreRegModels.empty()) {
      return TextError::err("no core registers to access");
    }
    if (!aBit) {
      CoreRegModel::RegIndex regIndex = mCoreRegModels[0]->regindexFromModbusReg(aAddress, aInput);
      if (aWrite) {
        // new data written, forward to hardware (SPI, Proxy)
        mCoreRegModels[0]->updateHardwareFromRegisterCache(regIndex, regIndex);
      }
      else {
        // get current data from core (via SPI or Proxy)
        mCoreRegModels[0]->updateRegisterCacheFromHardware(regIndex, regIndex);
      }
    }
    return err;
  }


  // MARK: - initialisation

  virtual void initialize()
  {
    ErrorPtr err;
    LOG(LOG_NOTICE, "kksdcmd: initialize");
    #if ENABLE_UBUS
    // start ubus API, if we have it
    if (mUbusApiServer) {
      mUbusApiServer->startServer();
    }
    #endif // ENABLE_UBUS
    #if ENABLE_P44SCRIPT
    // install app specific global predefined objects
    // - app specific functions
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new KksDcmDLookup(*this));
    #if ENABLE_UBUS
    // - web api implemented in p44script (webrequest() global event source)
    StandardScriptingDomain::sharedDomain().registerMemberLookup(gScriptApiLookupP);
    #endif
    // - generic function
    #if ENABLE_HTTP_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::HttpLookup);
    #endif // ENABLE_HTTP_SCRIPT_FUNCS
    #if ENABLE_SOCKET_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::SocketLookup);
    #endif // ENABLE_SOCKET_SCRIPT_FUNCS
    #if ENABLE_WEBSOCKET_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::WebSocketLookup);
    #endif // ENABLE_WEBSOCKET_SCRIPT_FUNCS
    #if ENABLE_ANALOGIO_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::AnalogIoLookup);
    #endif
    #if ENABLE_DIGITALIO_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::DigitalIoLookup);
    #endif
    #if ENABLE_DCMOTOR_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::DcMotorLookup);
    #endif
    #if ENABLE_I2C_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::I2CLookup());
    #endif
    #if ENABLE_SPI_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::SPILookup());
    #endif
    #if ENABLE_MODBUS_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::ModbusLookup);
    #endif // ENABLE_HTTP_SCRIPT_FUNCS
    #endif // ENABLE_P44SCRIPT

    // Set up local SPI core, if any
    int spino;
    if (getIntOption("corespi", spino)) {
      // make first generator access local core via SPI
      SPICoreRegModelPtr spicore = new SPICoreRegModel;
      mCoreRegModels.push_back(spicore);
      SPIDevicePtr dev = SPIManager::sharedManager().getDevice(spino, "generic");
      spicore->coreSPIProto().setSpiDevice(dev);
      // Set up modbus slave for access to local generator's registers
      string mbconn;
      if (getStringOption("modbus", mbconn)) {
        spicore->modbusSlave().setConnectionSpecification(mbconn.c_str(), DEFAULT_MODBUS_IP_PORT, NULL);
        spicore->modbusSlave().setSlaveId(string_format("KKS-DCM version %s", Application::version().c_str()));
        // start TCP server for modbus slave
        err = spicore->modbusSlave().connect();
        if (Error::notOK(err)) {
          LOG(LOG_ERR, "Error starting modbus TCP server/slave: %s", err->text());
        }
        // install modbus access handler
        spicore->modbusSlave().setValueAccessHandler(boost::bind(&KksDcmD::modbusAccessHandler, this, _1, _2, _3, _4));
      }
    }
    // set up core proxies
    string baseip;
    if (getStringOption("proxybaseip", baseip)) {
      uint32_t ip = stringToIpv4(baseip.c_str());
      int port = DEFAULT_MODBUS_IP_PORT;
      getIntOption("proxyport", port);
      if (ip==0) {
        err = TextError::err("Invalid proxy base IP address: %s", baseip.c_str());
      }
      else {
        // how many proxies?
        int numproxies = 1; // default to 1
        getIntOption("numproxies", numproxies);
        while (numproxies>0) {
          // create a proxy
          ProxyCoreRegModelPtr proxycore = new ProxyCoreRegModel;
          mCoreRegModels.push_back(proxycore);
          // set up modbus master
          proxycore->modbusMaster().setConnectionSpecification(ipv4ToString(ip).c_str(), port, NULL);
          // done with this one
          numproxies--;
          ip++; // next proxy will just have next consecutive IP
        }
      }
    }
    #if ENABLE_P44SCRIPT
    // load and start main script
    if (getStringOption("mainscript", mMainScriptFn)) {
      string code;
      err = string_fromfile(dataPath(mMainScriptFn), code);
      if (Error::notOK(err)) {
        err = string_fromfile(resourcePath(mMainScriptFn), code);
        if (Error::notOK(err)) {
          err->prefixMessage("Cannot open '%s': ", mMainScriptFn.c_str());
        }
      }
      if (Error::isOK(err)) {
        mMainScript.setSource(code);
        ScriptObjPtr res = mMainScript.syntaxcheck();
        if (res && res->isErr()) {
          err = res->errorValue();
          err->prefixMessage("Syntax Error in mainscript: ");
        }
        else {
          LOG(LOG_NOTICE, "Starting mainscript");
          mMainScript.run(inherit, boost::bind(&KksDcmD::mainScriptDone, this, _1));
        }
      }
    }
    #endif // ENABLE_P44SCRIPT
    // start cache refreshing
    int poll_ms = DEFAULT_POLL_INTERVAL_MS;
    getIntOption("pollinterval", poll_ms);
    mPollInterval = poll_ms*MilliSecond;
    if (mPollInterval!=0) {
      mPollTimer.executeOnce(boost::bind(&KksDcmD::corePoller, this), mPollInterval);
    }
    // display error
    if (Error::notOK(err)) {
      LOG(LOG_ERR, "Startup error: %s", Error::text(err));
    }
  }


void corePoller()
{
    // Tracks whether each generator was reachable in the last cycle
    static std::vector<bool> reachable(mCoreRegModels.size(), true);

    // Remembers which bad generator was retried last time
    static int lastBadTried = -1;

    // 1) Poll all generators that were previously marked as good
    for (int generator = 0; generator < mCoreRegModels.size(); generator++) {
        if (!reachable[generator]) continue;  // skip previously bad generators

        LOG(LOG_INFO, "\n=== polling GOOD generator #%d", generator);
        ErrorPtr err = mCoreRegModels[generator]->updateRegisterCache();

        if (Error::notOK(err)) {
            LOG(LOG_ERR, "error polling generator #%d: %s", generator, err->text());
            reachable[generator] = false;      // mark generator as bad
        }
    }

    // 2) Retry exactly one bad generator per cycle
    int badCount = 0;
    for (bool ok : reachable) if (!ok) badCount++;

    if (badCount > 0) {
        // Find the next bad generator in a round-robin manner
        for (int i = 0; i < mCoreRegModels.size(); i++) {
            lastBadTried = (lastBadTried + 1) % mCoreRegModels.size();
            if (!reachable[lastBadTried]) {
                int g = lastBadTried;
                LOG(LOG_INFO, "\n=== retry BAD generator #%d", g);

                ErrorPtr err = mCoreRegModels[g]->updateRegisterCache();
                if (Error::notOK(err)) {
                    LOG(LOG_ERR, "still bad: generator #%d: %s", g, err->text());
                } else {
                    LOG(LOG_INFO, "generator #%d recovered!", g);
                    reachable[g] = true;  // mark generator as good again
                }
                break; // retry only one bad generator per cycle
            }
        }
    }

    // 3) Schedule next poll cycle
    mPollTimer.executeOnce(
        boost::bind(&KksDcmD::corePoller, this),
        mPollInterval
    );
}



  void mainScriptDone(ScriptObjPtr aResult)
  {
    if (aResult && aResult->isErr()) {
      LOG(LOG_ERR, "mainscript failed: %s", aResult->errorValue()->text());
    }
    else {
      LOG(LOG_NOTICE, "mainscript terminated with result: %s", ScriptObj::describe(aResult).c_str());
    }
  }


  void delayedTerminate(int aExitCode)
  {
    terminateApp(aExitCode);
  }

};


// MARK: - script functions

// FIXME: probably remove exit(), we now have restartapp() in p44script

// exit(exitcode)
FUNC_ARG_DEFS(exit, { numeric } );
static void exit_func(BuiltinFunctionContextPtr f)
{
  KksDcmD& kksdcmd = static_cast<KksDcmDLookup*>(f->funcObj()->getMemberLookup())->mKksdcmd;
  kksdcmd.terminateApp(f->arg(0)->intValue());
  f->finish();
}



static const BuiltinMemberDescriptor kksdcmdGlobals[] = {
  FUNC_DEF_W_ARG(exit, executable|null),
  { NULL } // terminator
};


KksDcmDLookup::KksDcmDLookup(KksDcmD &aKksdcmd) :
  inherited(kksdcmdGlobals),
  mKksdcmd(aKksdcmd)
{
}


// MARK: - main

int main(int argc, char **argv)
{
  // prevent all logging until command line determines level
  SETLOGLEVEL(LOG_EMERG);
  SETERRLEVEL(LOG_EMERG, false); // messages, if any, go to stderr

  // create app with current mainloop
  KksDcmD *application = new(KksDcmD);
  // pass control
  int status = application->main(argc, argv);
  // done
  delete application;
  return status;
}
