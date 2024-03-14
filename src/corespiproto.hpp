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

#ifndef __kksdcmd__corespiproto__
#define __kksdcmd__corespiproto__

#include "p44utils_common.hpp"
#include "spi.hpp"

using namespace std;

namespace p44 {

  class CoreSPIError : public Error
  {
  public:
    typedef enum {
      OK,
      noSPI, ///< no SPI device set
      writeErr, ///< error writing
      readErr, ///< error reading
      readTimeout, ///< too many fill bytes
      crcErr, ///< CRC error in received data
      protoErr, ///< Protocol error
      numErrorCodes
    } ErrorCodes;
    static const char *domain() { return "CoreSPI"; }
    virtual const char *getErrorDomain() const P44_OVERRIDE { return CoreSPIError::domain(); };
    CoreSPIError(ErrorCodes aError) : Error(ErrorCode(aError)) {};
    #if ENABLE_NAMED_ERRORS
  protected:
    virtual const char* errorName() const P44_OVERRIDE { return errNames[getErrorCode()]; };
  private:
    static constexpr const char* const errNames[numErrorCodes] = {
      "OK",
      "noSPI",
      "writeErr",
      "readErr",
      "readTimeout",
      "crcErr",
      "protoErr",
    };
    #endif // ENABLE_NAMED_ERRORS
  };


  class CoreSPIProto : public P44LoggingObj
  {
    typedef P44LoggingObj inherited;

    SPIDevicePtr mSPI;

  public:

    CoreSPIProto();
    virtual ~CoreSPIProto();

    /// Specify the SPI device to use for accessing the SPI bus
    void setSpiDevice(SPIDevicePtr aSPIDevice) { mSPI = aSPIDevice; };

    /// Write Data
    /// @param aAddr the data bank address to start writing
    /// @param aLen the number of bytes to write
    /// @param aData the data to write
    /// @return OK or error
    ErrorPtr writeData(uint16_t aAddr, uint8_t aLen, const uint8_t* aData);

    /// Read data
    /// @param aAddr the data bank address to start reading
    /// @param aLen the number of bytes to read
    /// @param aData data buffer to place read data into
    /// @return OK or error
    ErrorPtr readData(uint16_t aAddr, uint8_t aLen, uint8_t* aData);

    /// CRC16
    static void crc16addbyte(uint16_t &aCrc16, uint8_t aByte);
    static uint16_t crc16(uint16_t aCrc, uint8_t aLen, const uint8_t* aData);




  };
  typedef boost::intrusive_ptr<CoreSPIProto> CoreSPIProtoPtr;

} // namespace p44

#endif // __kksdcmd__corespiproto__
