//
//  main.cpp
//  p44mbc
//
//  Created by Lukas Zeller on 2019-04-26
//  Copyright (c) 2019 plan44.ch. All rights reserved.
//

#include "application.hpp"

#include "macaddress.hpp"
#include "modbus.hpp"
#include "utils.hpp"
#include "jsonobject.hpp"
#include "analogio.hpp"
#include "gpio.hpp"

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


#if ENABLE_P44SCRIPT && ENABLE_UBUS && TO_BE_DONE

// MARK: - ApiRequestObj

class ApiRequestObj : public JsonValue
{
  typedef JsonValue inherited;

  EventSource* mEventSource;
  UbusRequestPtr mRequest;

public:
  ApiRequestObj(UbusRequestPtr aRequest, EventSource* aApiEventSource) :
    inherited(aRequest ? aRequest->getRequest() : JsonObjectPtr()),
    mRequest(aRequest),
    mEventSource(aApiEventSource)
  {
  }

  void sendResponse(JsonObjectPtr aResponse, ErrorPtr aError)
  {
    if (mRequest) mRequest->sendResponse(aResponse, aError);
    mRequest.reset(); // done now
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

static ScriptApiLookup* scriptApiLookupP; // FIXME: ugly

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
  friend class P44FeatureD;

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
  f->finish(new ApiRequestObj(scriptApiLookupP->pendingRequest(), scriptApiLookupP));
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

  // modbus
  ModbusSlavePtr modBusSlave; ///< modbus slave
  ModbusMasterPtr modBusMaster; ///< modbus master
  DigitalIoPtr modbusRxEnable; ///< if set, modbus receive is enabled

  // app
  bool mActive;

  #if ENABLE_P44SCRIPT
  // scripting
  string mMainScriptFn; ///< filename for the main script
  ScriptSource mMainScript;
  ScriptMainContextPtr mScriptMainContext;
  #endif // ENABLE_P44SCRIPT

public:

  KksDcmD() :
    mMainScript(sourcecode+regular, "main") // only init script may have declarations
  {
    mActive = true;
    // let all scripts run in the same context

    mScriptMainContext = mMainScript.domain()->newContext();
    mMainScript.setSharedMainContext(mScriptMainContext);
  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
    "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "connection",      true,  "connspec;serial interface for RTU or IP address for TCP (/device or IP[:port])" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable, 'RTS' or 'RS232'" },
      { 0  , "rs485txdelay",    true,  "delay;delay of tx enable signal in uS" },
      { 0  , "rs485rxenable",   true,  "pinspec;a digital output pin specification for RX input enable" },
      { 0  , "bytetime",        true,  "time;custom time per byte in nS" },
      { 0  , "slave",           true,  "slave;use this slave by default (0: act as master)" },
      { 0  , "debugmodbus",     false, "enable libmodbus debug messages to stderr" },
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
    UbusObjectPtr u = new UbusObject("kksdcmd", boost::bind(&KksDcmD::ubusApiRequestHandler, this, _1, _2, _3));
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


  void ubusApiRequestHandler(UbusRequestPtr aUbusRequest, const string aMethod, JsonObjectPtr aJsonRequest)
  {
    if (aMethod=="log") {
      if (aJsonRequest) {
        JsonObjectPtr o;
        if (aJsonRequest->get("level", o)) {
          int oldLevel = LOGLEVEL;
          int newLevel = o->int32Value();
          SETLOGLEVEL(newLevel);
          LOG(newLevel, "\n\n========== changed log level from %d to %d ===============", oldLevel, newLevel);
        }
        if (aJsonRequest->get("deltastamps", o)) {
          SETDELTATIME(o->boolValue());
        }
      }
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aMethod=="quit") {
      LOG(LOG_WARNING, "terminated via UBUS quit method");
      terminateApp(1);
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aMethod=="api") {
      ErrorPtr err;
      JsonObjectPtr result;
      JsonObjectPtr o;
      if (aJsonRequest) {
        JsonObjectPtr subsys;
        if (aJsonRequest->get("modbus", subsys)) {
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
            if (aJsonRequest->get("reg", o)) reg = o->int32Value();
            if (aJsonRequest->get("count", o)) numReg = o->int32Value();
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
            if (aJsonRequest->get("reg", o)) reg = o->int32Value();
            int numReg = 0;
            uint16_t tab_reg[MAX_REG];
            if (reg<0) {
              err = TextError::err("invalid reg=%d");
            }
            else {
              if (aJsonRequest->get("values", o)) {
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
        #if ENABLE_P44SCRIPT
        if (aJsonRequest->get("mainscript", subsys)) {
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
        #ifdef TO_BE_DONE
        else if (aJsonRequest->get("scriptapi", subsys)) {
          // scripted parts of the (web) API
          if (!scriptApiLookup.hasSinks()) {
            // no script API active
            err = WebError::webErr(500, "script API not active");
          }
          else {
            %%%
            scriptApiLookup.mPendingScriptApiRequest = UbusRequestPtr(new APICallbackRequest(aData, aRequestDoneCB));
            scriptApiLookup.sendEvent(new ApiRequestObj(scriptApiLookup.mPendingScriptApiRequest, &scriptApiLookup));
          }
        }
        #endif // TO_BE_DONE
        #endif // ENABLE_P44SCRIPT
      }
      else {
        err = TextError::err("missing command object");
      }
      JsonObjectPtr response = JsonObject::newObj();
      response->add("result", result); // including empty result
      if (err) response->add("error", JsonObject::newString(err->description()));
      aUbusRequest->sendResponse(response);
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
    #if TO_BE_DONE
    // slave address (or master when address==255)
    int slave = 1;
    getIntOption("slave", slave);
    // General modbus connection params
    string mbconn;
    if (!getStringOption("connection", mbconn)) {
      terminateAppWith(TextError::err("must specify --connection"));
      return;
    }
    string txen;
    getStringOption("rs485txenable", txen);
    int txDelayUs = Never;
    getIntOption("rs485txdelay", txDelayUs);
    int byteTimeNs = 0;
    getIntOption("bytetime", byteTimeNs);
    string rxen;
    getStringOption("rs485rxenable", rxen);
    bool modbusDebug = getOption("debugmodbus");
    // Master or slave
    if (slave!=0) {
      // we are a modbus slave
      modBusSlave = ModbusSlavePtr(new ModbusSlave);
      err = modBusSlave->setConnectionSpecification(
        mbconn.c_str(),
        DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
        txen.c_str(), txDelayUs,
        rxen.empty() ? NULL : rxen.c_str(), // NULL if there is no separate rx enable
        byteTimeNs
      );
      if (Error::notOK(err)) {
        terminateAppWith(err->withPrefix("Invalid modbus connection: "));
        return;
      }
      modBusSlave->setSlaveAddress(slave);
      modBusSlave->setSlaveId(string_format("p44mbc %s %06llX", version().c_str(), macAddress()));
      modBusSlave->setDebug(modbusDebug);
      // registers
      modBusSlave->setRegisterModel(
        0, 0, // coils
        0, 0, // input bits
        REGISTER_FIRST, REGISTER_LAST-REGISTER_FIRST+1, // registers
        0, 0 // input registers
      );
      // connect
      err = modBusSlave->connect();
      if (Error::notOK(err)) {
        terminateAppWith(err->withPrefix("Failed to start modbus slave server: "));
        return;
      }
      // - modbus slave scripting functions
      StandardScriptingDomain::sharedDomain().registerMember("modbus_slave", modBusSlave->representingScriptObj());
    }
    else {
      // Modbus master
      modBusMaster = ModbusMasterPtr(new ModbusMaster);
      err = modBusMaster->setConnectionSpecification(
        mbconn.c_str(),
        DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
        txen.c_str(), txDelayUs,
        rxen.empty() ? NULL : rxen.c_str(), // NULL if there is no separate rx enable
        byteTimeNs
      );
      if (Error::notOK(err)) {
        terminateAppWith(err->withPrefix("Invalid modbus connection: "));
        return;
      }
      modBusMaster->setDebug(modbusDebug);
      // - modbus master scripting functions
      StandardScriptingDomain::sharedDomain().registerMember("modbus_master", modBusMaster->representingScriptObj());
    }
    #endif // TO_BE_DONE
    #if ENABLE_P44SCRIPT
    // install app specific global predefined objects
    // - app specific functions
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new KksDcmDLookup(*this));
    // - generic function
    #if ENABLE_MODBUS_SCRIPT_FUNCS
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44Script::ModbusLookup);
    #endif // ENABLE_HTTP_SCRIPT_FUNCS
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
    // TODO: add these
    //StandardScriptingDomain::sharedDomain().registerMemberLookup(new i2cLookup());
    //StandardScriptingDomain::sharedDomain().registerMemberLookup(new spiLookup());
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
