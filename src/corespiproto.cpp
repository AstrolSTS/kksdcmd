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

// File scope debugging options
// - Set ALWAYS_DEBUG to 1 to enable DBGLOG output even in non-DEBUG builds of this file
#define ALWAYS_DEBUG 0
// - set FOCUSLOGLEVEL to non-zero log level (usually, 5,6, or 7==LOG_DEBUG) to get focus (extensive logging) for this file
//   Note: must be before including "logger.hpp" (or anything that includes "logger.hpp")
#define FOCUSLOGLEVEL 0

#include "corespiproto.hpp"

using namespace p44;

CoreSPIProto::CoreSPIProto()
{

}

CoreSPIProto::~CoreSPIProto()
{

}


ErrorPtr CoreSPIProto::writeData(uint16_t aAddr, uint8_t aLen, const uint8_t* aData)
{
  if (!mSPI) return new CoreSPIError(CoreSPIError::noSPI);
  uint8_t wrhdr[5];
  wrhdr[0] = 0xAB; // lead in
  wrhdr[1] = 0x01; // write cmd
  wrhdr[2] = aAddr & 0xFF; // addr LSB
  wrhdr[3] = (aAddr>>8) & 0xFF; // addr MSB
  wrhdr[4] = aLen; // len
  uint16_t crc = crc16(0, 5, wrhdr);
  crc = crc16(crc, aLen, aData);
  // send header
  if (mSPI->SPIRawWriteRead(5, wrhdr, 0, NULL, false, true)) { // keep transaction running
    // send data
    if (mSPI->SPIRawWriteRead(aLen, aData, 0, NULL, false, true)) { // keep transaction running
      // send CRC
      wrhdr[0] = crc & 0xFF; // crc LSB
      wrhdr[1] = (crc>>8) & 0xFF; // crc MSB
      if (mSPI->SPIRawWriteRead(2, wrhdr, 0, NULL, false, false)) { // transaction ends here
        // successful write
        return ErrorPtr();
      }
    }
  }
  // something went wrong
  return new CoreSPIError(CoreSPIError::writeErr);
}


ErrorPtr CoreSPIProto::readData(uint16_t aAddr, uint8_t aLen, uint8_t* aData)
{
  ErrorPtr err;
  if (!mSPI) return new CoreSPIError(CoreSPIError::noSPI);
  uint8_t rdhdr[5];
  rdhdr[0] = 0xAB; // lead in
  rdhdr[1] = 0x02; // read cmd
  rdhdr[2] = aAddr & 0xFF; // addr LSB
  rdhdr[3] = (aAddr>>8) & 0xFF; // addr MSB
  rdhdr[4] = aLen; // len
  uint16_t crc = crc16(0, 5, rdhdr);
  uint8_t buf[256];
  // send header and start reading
  // - minimally, we'll get the expected number of bytes + lead in
  // - we'll also get 2 bytes CRC, but we read those separately to end the transaction
  uint8_t expected = aLen+1;
  if (!mSPI->SPIRawWriteRead(5, rdhdr, expected, buf, false, true)) { // keep transaction running
    err = Error::err<CoreSPIError>(CoreSPIError::readErr, "failed initiating read");
  }
  else {
    // must find a lead-in first
    bool datastarted = false;
    int maxreps = 100;
    while (aLen>0) {
      uint8_t i = 0;
      while (!datastarted && i<expected) {
        if (buf[i]==0xAB) {
          // real data starts at i+1
          crc16addbyte(crc, 0xAB);
          datastarted = true;
          i++;
          break;
        }
        else if (buf[i]!=0xFF) {
          // should be delay byte but isn't
          err = Error::err<CoreSPIError>(CoreSPIError::protoErr, "invalid read delay filler byte: 0x%02X", buf[i]);
          break;
        }
        // delay byte, just swallow
        i++;
      }
      // transfer the real data (if any)
      while (datastarted && i<expected && aLen>0) {
        crc16addbyte(crc, buf[i]);
        *aData++ = buf[i];
        aLen--;
        i++;
      }
      if (err || aLen<=0) break; // all data received
      // more data to read: if data not yet started, including lead-in byte
      expected = aLen + (datastarted ? 0 : 1);
      if (maxreps-- <=0) {
        err = Error::err<CoreSPIError>(CoreSPIError::readTimeout, "read preamble too long");
        break;
      }
      // read more
      if (!mSPI->SPIRawWriteRead(0, NULL, expected, buf, false, true)) { // keep transaction running
        err = Error::err<CoreSPIError>(CoreSPIError::readErr, "failed reading more data");
      }
    }
    // now read and check CRC
    if (!mSPI->SPIRawWriteRead(0, NULL, 2, buf, false, false)) { // end transaction here
      err = Error::err<CoreSPIError>(CoreSPIError::readErr, "failed reading CRC bytes");
    }
    else {
      // compare CRC
      uint16_t recCrc = buf[0] + (((uint16_t)buf[1])<<8);
      if (!err && recCrc!=crc) {
        err = Error::err<CoreSPIError>(CoreSPIError::crcErr, "read CRC mismatch, found=0x%02X, expected=0x%02X", recCrc, crc);
      }
    }
  }
  return err;
}


static const uint16_t CRC16_polynominal = 0x8408;

void CoreSPIProto::crc16addbyte(uint16_t &aCrc16, uint8_t aByte)
{
  int i;
  for(i=8; i; i--) {
    if ((aByte ^ aCrc16) & 1) {
      aCrc16 >>= 1;
      aCrc16 ^= CRC16_polynominal;
    }
    else {
      aCrc16 >>= 1;
    }
    aByte>>=1;
  }
}


uint16_t CoreSPIProto::crc16(uint16_t aCrc, uint8_t aLen, const uint8_t* aData)
{
  while (aLen-- > 0) {
    crc16addbyte(aCrc, *aData++);
  }
  return aCrc;
}
