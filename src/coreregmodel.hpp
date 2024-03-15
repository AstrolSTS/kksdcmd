//
//  Copyright (c) 2022 plan44.ch / Lukas Zeller, Zurich, Switzerland
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

#ifndef __kksdcmd__coreregmodel__
#define __kksdcmd__coreregmodel__

#include "p44utils_common.hpp"
#include "corespiproto.hpp"
#include "modbus.hpp"
#include "jsonobject.hpp"

using namespace std;

namespace p44 {

  class CoreRegError : public Error
  {
  public:
    typedef enum {
      OK,
      invalidIndex, ///< invalid register index
      readOnly, ///< register is read-only
      outOfRange, ///< value provided is out of range
      invalidInput, ///< register (string) value provided is not valud
      numErrorCodes
    } ErrorCodes;
    static const char *domain() { return "CoreReg"; }
    virtual const char *getErrorDomain() const P44_OVERRIDE { return CoreRegError::domain(); };
    CoreRegError(ErrorCodes aError) : Error(ErrorCode(aError)) {};
    #if ENABLE_NAMED_ERRORS
  protected:
    virtual const char* errorName() const P44_OVERRIDE { return errNames[getErrorCode()]; };
  private:
    static constexpr const char* const errNames[numErrorCodes] = {
      "OK",
      "invalidIndex",
      "readOnly",
      "outOfRange",
      "invalidInput",
    };
    #endif // ENABLE_NAMED_ERRORS
  };



  class CoreRegModel : public P44LoggingObj
  {
    typedef P44LoggingObj inherited;

  public:

    CoreRegModel();
    virtual ~CoreRegModel();

    typedef uint16_t RegIndex;

    /// @return highest register index
    RegIndex maxReg();

    /// @param aModbusReg the modbus register number
    /// @param aInput set if this is a read-only input register
    /// @return valid regindex if corresponding modbus register exists,
    ///   invalid index that will fail in all other calls otherwise
    RegIndex regindexFromModbusReg(int aModbusReg, bool aInput);

    /// @param aRegName the register name (case insensitive)
    /// @return valid regindex if corresponding register exists,
    ///   invalid index that will fail in all other calls otherwise
    RegIndex regindexFromRegName(const string aRegName);


    /// get user facing register value (scaled to real world units) internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue receives the real world value
    /// @return OK or error
    /// @note no automatic transfer from actual hardware or proxied device happens, just accessing cached register data as-is
    ErrorPtr getUserValue(RegIndex aRegIdx, double& aValue);

    /// set user facing register value (scaled to real world units) to internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue the new value, will be checked against min/max and NOT set if out of range
    /// @return OK or error, in particular out-of-range
    /// @note no automatic transfer to actual hardware or proxied device happens, just writing to register cache
    ErrorPtr setUserValue(RegIndex aRegIdx, double aValue);

    /// get user facing infos for a register
    /// @param aRegIdx the register index (internal)
    /// @return json object with all info for the register, or NULL if register does not exist
    JsonObjectPtr getRegisterInfo(RegIndex aRegIdx);

    /// set user facing value into a register
    /// @param aRegIdx the register index (internal)
    /// @param aNewValue the new value (usually double, but might also be string)
    /// @return OK or error, in particular syntax errors and out-of-range
    ErrorPtr setRegisterValue(RegIndex aRegIdx, JsonObjectPtr aNewValue);

    /// get user facing infos for all registers
    /// @return json array with all info for all registers
    JsonObjectPtr getRegisterInfos();

    /// refresh the entire cache (all registers)
    ErrorPtr updateRegisterCache();


    /// @name register model and underlying hardware access implementation
    /// @{

    /// get engineering register value (with correct sign) from internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue receives the (possibly signed) engineering value
    /// @return OK or error
    /// @note no automatic transfer from actual hardware or proxied device happens, just accessing cached register data as-is
    virtual ErrorPtr getEngineeringValue(RegIndex aRegIdx, int32_t& aValue) = 0;

    /// set engineering register value (with correct sign) to internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue the new engineering value, if aUserInput==true, this will be checked against min/max and NOT set if out of range
    /// @param aUserInput if set, aValue is considered user input and will be rejected when the register is read-only or value is out of range
    /// @return OK or error, in particular out-of-range
    /// @note no automatic transfer to actual hardware or proxied device happens, just writing to register cache
    virtual ErrorPtr setEngineeringValue(RegIndex aRegIdx, int32_t aValue, bool aUserInput) = 0;

    /// update register cache from local or proxied hardware
    /// @param aFromIdx first register index to read (internal)
    /// @param aToIdx last register to read
    /// @return OK or error
    virtual ErrorPtr updateRegisterCacheFromHardware(RegIndex aFromIdx, RegIndex aToIdx) = 0;

    /// update local or proxied hardware from register cache
    /// @param aRegIdx register index to write (internal)
    /// @return OK or error
    virtual ErrorPtr updateHardwareFromRegisterCache(RegIndex aRegIdx) = 0;

    /// @}

  protected:

    ErrorPtr checkUserInput(RegIndex aRegIdx, int32_t aValue);

  };
  typedef boost::intrusive_ptr<CoreRegModel> CoreRegModelPtr;



  class SPICoreRegModel : public CoreRegModel
  {
    typedef CoreRegModel inherited;

    ModbusSlavePtr mModbusSlave; // modbus exposure of local SPI registers
    CoreSPIProtoPtr mCoreSPIProto; // SPI protocol for accessing local registers

  public:

    SPICoreRegModel();
    virtual ~SPICoreRegModel();

    /// access the modbus slave (mainly to set connection specs)
    ModbusSlave& modbusSlave();

    /// access the SPI core protocol handler (mainly to set actual SPI device to use)
    CoreSPIProto& coreSPIProto();

    /// @name register model and underlying hardware access implementation
    /// @{

    /// get engineering register value (with correct sign) from internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue receives the (possibly signed) engineering value
    /// @return OK or error
    /// @note no automatic transfer from actual hardware or proxied device happens, just accessing cached register data as-is
    virtual ErrorPtr getEngineeringValue(RegIndex aRegIdx, int32_t& aValue) P44_OVERRIDE;


    /// set engineering register value (with correct sign) to internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue the new engineering value, if aUserInput==true, this will be checked against min/max and NOT set if out of range
    /// @param aUserInput if set, aValue is considered user input and will be rejected when the register is read-only or value is out of range
    /// @return OK or error, in particular out-of-range
    /// @note no automatic transfer to actual hardware or proxied device happens, just writing to register cache
    virtual ErrorPtr setEngineeringValue(RegIndex aRegIdx, int32_t aValue, bool aUserInput) P44_OVERRIDE;

    /// update register cache from local or proxied hardware
    /// @param aFromIdx first register index to read (internal)
    /// @param aToIdx last register to read
    /// @return OK or error
    virtual ErrorPtr updateRegisterCacheFromHardware(RegIndex aFromIdx, RegIndex aToIdx) P44_OVERRIDE;

    /// update local or proxied hardware from register cache
    /// @param aRegIdx register index to write (internal)
    /// @return OK or error
    virtual ErrorPtr updateHardwareFromRegisterCache(RegIndex aRegIdx) P44_OVERRIDE;

    /// @}

  private:

    /// read a range of SPI registers into presented buffer space
    /// @param aFromIdx first register index to read (internal)
    /// @param aToIdx input: last register to read, output: last register actually read
    /// @param aBuffer memory to read into
    /// @param aBufSize size of buffer
    /// @return OK or error
    ErrorPtr readSPIRegRange(RegIndex aFromIdx, RegIndex &aToIdx, uint8_t* aBuffer, size_t aBufSize);

    /// @param aRegIdx the register index (internal)
    /// @param aData will receive the register's contents
    /// @param aBuffer memory contai  ning raw SPI register data
    /// @param aFirstRegIdx index of first register represented by data in aBuffer
    /// @param aLastRegIdx index of last register represented by data in aBuffer
    /// @return OK or error
    ErrorPtr readRegFromBuffer(RegIndex aRegIdx, int32_t &aData, uint8_t* aBuffer, RegIndex aFirstRegIdx, RegIndex aLastRegIdx);


    /// read single SPI register
    /// @param aRegIdx the register index (internal)
    /// @param aData will receive the register's contents
    /// @return OK or error
    ErrorPtr readSPIReg(RegIndex aRegIdx, int32_t &aData);

    /// write single SPI register
    /// @param aRegIdx the register index (internal)
    /// @param aData the new register value
    /// @return OK or error
    ErrorPtr writeSPIReg(RegIndex aRegIdx, int32_t aData);

  };
  typedef boost::intrusive_ptr<SPICoreRegModel> SPICoreRegModelPtr;



  class ProxyCoreRegModel : public CoreRegModel
  {
    typedef CoreRegModel inherited;

    ModbusMasterPtr mModbusMaster;

    typedef std::vector<int32_t> RegisterValueVector; // simple register value storage
    RegisterValueVector mRegisterValues;

  public:

    ProxyCoreRegModel();
    virtual ~ProxyCoreRegModel();

    /// access the modbus slave (mainly to set connection specs)
    ModbusMaster& modbusMaster();


    /// @name register model and underlying hardware access implementation
    /// @{

    /// get engineering register value (with correct sign) from internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue receives the (possibly signed) engineering value
    /// @return OK or error
    /// @note no automatic transfer from actual hardware or proxied device happens, just accessing cached register data as-is
    virtual ErrorPtr getEngineeringValue(RegIndex aRegIdx, int32_t& aValue) P44_OVERRIDE;

    /// set engineering register value (with correct sign) to internal register cache
    /// @param aRegIdx the register index (internal)
    /// @param aValue the new engineering value, if aUserInput==true, this will be checked against min/max and NOT set if out of range
    /// @param aUserInput if set, aValue is considered user input and will be rejected when the register is read-only or value is out of range
    /// @return OK or error, in particular out-of-range
    /// @note no automatic transfer to actual hardware or proxied device happens, just writing to register cache
    virtual ErrorPtr setEngineeringValue(RegIndex aRegIdx, int32_t aValue, bool aUserInput) P44_OVERRIDE;

    /// update register cache from local or proxied hardware
    /// @param aFromIdx first register index to read (internal)
    /// @param aToIdx last register to read
    /// @return OK or error
    virtual ErrorPtr updateRegisterCacheFromHardware(RegIndex aFromIdx, RegIndex aToIdx) P44_OVERRIDE;

    /// update local or proxied hardware from register cache
    /// @param aRegIdx register index to write (internal)
    /// @return OK or error
    virtual ErrorPtr updateHardwareFromRegisterCache(RegIndex aRegIdx) P44_OVERRIDE;

    /// @}

  private:

    ErrorPtr modbusReadRegisterSequence(RegIndex aFromIdx, int aNumModbusRegs, bool& aConnected);

  };
  typedef boost::intrusive_ptr<ProxyCoreRegModel> ProxyCoreRegModelPtr;




} // namespace p44

#endif // __kksdcmd__coreregmodel__
