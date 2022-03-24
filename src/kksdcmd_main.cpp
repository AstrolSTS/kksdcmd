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

#define DEFAULT_MODBUS_RTU_PARAMS "115200,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
#define DEFAULT_MODBUS_IP_PORT 1502

#define REGISTER_FIRST 100
#define REGISTER_LAST 199

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

class ApiRequestObj : public JsonValue
{
  typedef JsonValue inherited;

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

  virtual EventSource *eventSource() const P44_OVERRIDE
  {
    return mEventSource;
  }

  virtual const ScriptObjPtr memberByName(const string aName, TypeInfo aMemberAccessFlags = none) P44_OVERRIDE;

};

// answer([answer value])        answer the request
static const BuiltInArgDesc answer_args[] = { { any|optionalarg } };
static const size_t answer_numargs = sizeof(answer_args)/sizeof(BuiltInArgDesc);
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
  { "answer", executable|any, answer_numargs, answer_args, &answer_func };


const ScriptObjPtr ApiRequestObj::memberByName(const string aName, TypeInfo aMemberAccessFlags)
{
  ScriptObjPtr val;
  if (uequals(aName, "answer")) {
    val = new BuiltinFunctionObj(&answer_desc, this, NULL);
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
  { "webrequest", executable|json|null, 0, NULL, &webrequest_func },
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

  // FIXME: remove
  /*
  // SPI
  CoreSPIProtoPtr mCoreSPI;
  */

  CoreRegModelPtr mCoreRegModel;

  // app
  bool mActive;

  #if ENABLE_P44SCRIPT
  // scripting
  string mMainScriptFn; ///< filename for the main script
  ScriptSource mMainScript;
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
      { 0  , "mainscript",     true,  "p44scriptfile;the main script to run after startup" },
      #endif
      #if ENABLE_UBUS
      { 0  , "ubusapi",         false, "enable ubus API" },
      #endif
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
    else if (aUbusRequest->method()=="api") {
      ErrorPtr err;
      JsonObjectPtr result;
      JsonObjectPtr o;
      if (aUbusRequest->msg()) {
        JsonObjectPtr subsys;
        /* example how a API for C++ level instantiated modbus slave access could work (from JENNY)
        // TODO: remove these???
        if (aUbusRequest->msg()->get("modbus", subsys)) {
          // modbus commands
          string cmd = subsys->stringValue();
          if (cmd=="debug_on") {
            if (modBusSlave) modBusSlave->setDebug(true);
            if (modBusMaster) modBusMaster->setDebug(true);
          }
          else if (cmd=="debug_off") {
            if (modBusSlave) modBusSlave->setDebug(false);
            if (modBusMaster) modBusMaster->setDebug(false);
          }
          else if (modBusSlave && cmd=="read_registers") {
            int reg = -1;
            int numReg = 1;
            if (aUbusRequest->msg()->get("reg", o)) reg = o->int32Value();
            if (aUbusRequest->msg()->get("count", o)) numReg = o->int32Value();
            if (reg<0 || numReg<1 || numReg>=MAX_REG) {
              err = TextError::err("invalid reg=%d, count=%d combination", reg, numReg);
            }
            else {
              uint16_t tab_reg[MAX_REG];
              for (int i=0; i<numReg; i++) {
                result->arrayAppend(JsonObject::newInt32(modBusSlave->getReg(reg+i, false)));
              }
            }
          }
          else if (modBusSlave && cmd=="write_registers") {
            int reg = -1;
            if (aUbusRequest->msg()->get("reg", o)) reg = o->int32Value();
            int numReg = 0;
            uint16_t tab_reg[MAX_REG];
            if (reg<0) {
              err = TextError::err("invalid reg=%d");
            }
            else {
              if (aUbusRequest->msg()->get("values", o)) {
                if (o->isType(json_type_array)) {
                  // multiple
                  for(int i=0; i<o->arrayLength(); i++) {
                    modBusSlave->setReg(reg+i, false, o->arrayGet(i)->int32Value());
                  }
                }
                else {
                  // single
                  modBusSlave->setReg(reg, false, o->int32Value());
                }
              }
              else {
                err = TextError::err("missing 'values'");
              }
            }
            if (Error::isOK(err)) {
              result = JsonObject::newBool(true);
            }
          }
          else {
            err = TextError::err("unknown modbus command");
          }
        }
        */
        #if ENABLE_P44SCRIPT
        // API for editing/starting/stopping mainscript via web interface
        if (aUbusRequest->msg()->get("mainscript", subsys)) {
          if (subsys->get("execcode", o)) {
            // direct execution of a script command line in the common main/initscript context
            ScriptSource src(sourcecode+regular+keepvars+concurrently+ephemeralSource, "execcode");
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

    /*
    // FIXME: clean up
    mCoreSPI = CoreSPIProtoPtr(new CoreSPIProto);
    SPIDevicePtr dev = SPIManager::sharedManager().getDevice(10, "generic");
    mCoreSPI->setSpiDevice(dev);

    uint8_t data[3];
    data[0] = 0xC8;
    data[1] = 0x00;
    err = mCoreSPI->writeData(0x0036, 2, data);
    LOG(LOG_NOTICE, "write: status %s", Error::text(err))

    dev->getBus().setDataToRead(hexToBinaryString("FFFFAB9400003367"));
    err = mCoreSPI->readData(0x0020, 3, data);
    string res;
    res.assign((char *)data, 3);
    LOG(LOG_NOTICE, "read: %s: status %s", binaryToHexString(res).c_str(), Error::text(err))

    terminateApp(0);
    */

    // FIXME: clean up
    mCoreRegModel = CoreRegModelPtr(new CoreRegModel);
    SPIDevicePtr dev = SPIManager::sharedManager().getDevice(10, "generic");
    mCoreRegModel->coreSPIProto().setSpiDevice(dev);
    mCoreRegModel->modbusSlave().setConnectionSpecification("0.0.0.0:8765", 8765, NULL);
    err = mCoreRegModel->updateModbusRegistersFromSPI(0, mCoreRegModel->maxReg());
    if (Error::notOK(err)) {
      LOG(LOG_ERR, "Error updating registers: %s", err->text());
    }
    JsonObjectPtr infos = mCoreRegModel->getRegisterInfos();
    LOG(LOG_NOTICE, "Registers:\n%s", infos->json_str(JSON_C_TO_STRING_PRETTY).c_str());
    terminateApp(0);


    // TODO: add hardwired init
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
    // display error
    if (Error::notOK(err)) {
      LOG(LOG_ERR, "Startup error: %s", Error::text(err));
    }
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


// exit(exitcode)
static const BuiltInArgDesc exit_args[] = { { numeric } };
static const size_t exit_numargs = sizeof(exit_args)/sizeof(BuiltInArgDesc);
static void exit_func(BuiltinFunctionContextPtr f)
{
  KksDcmD& kksdcmd = static_cast<KksDcmDLookup*>(f->funcObj()->getMemberLookup())->mKksdcmd;
  kksdcmd.terminateApp(f->arg(0)->intValue());
  f->finish();
}



static const BuiltinMemberDescriptor kksdcmdGlobals[] = {
  { "exit", executable|null, exit_numargs, exit_args, &exit_func },
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
