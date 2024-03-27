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

#include "coreregmodel.hpp"

#include "valueunits.hpp"

#include <math.h>

using namespace p44;

// MARK: - Core Module Register definitions

enum {
  reg_byte = 0x1,
  reg_word = 0x2,
  reg_triplet = 0x3,
  reg_long = 0x4,
  reg_bytecount_mask = 0xF,
  reg_signed = 0x100,
  // combinations
  reg_uint8 = reg_byte,
  reg_sint8 = reg_byte|reg_signed,
  reg_uint16 = reg_word,
  reg_sint16 = reg_word|reg_signed,
  reg_uint24 = reg_triplet,
};
typedef uint16_t RegisterLayout;


typedef struct {
  const char *regname; ///< register name
  const char *description; ///< description
  long min; ///< signed minimum engineering value
  long max; ///< signed maximum engineering value
  double resolution; ///< resolution of one engineering value count (when expressed in ValueUnit as specified with "unit")
  ValueUnit unit; ///< value unit
  // SPI side
  uint16_t addr; ///< address of the first byte
  uint8_t rawlen; ///< number of raw bytes occupied
  RegisterLayout layout; ///< register layout
  // Modbus side
  uint16_t mbreg; ///< modbus register number
  bool mbinput; ///< modbus input register
} CoreModuleRegister;

// Modbus register layout constants
// - R/W registers
const int mbreg_first = 1;
const int mb_numregs = 233-mbreg_first+1;
// - Read-only (input) registers
const int mbinp_first = 1;
const int mb_numinps = 250-mbinp_first+1;

// Core module register definitions
const CoreModuleRegister coreModuleRegisterDefs[] = {
  // regname                      description                                 min,     max,       resolution, unit,                                addr,         rawlen,  layout,       mbreg, mbinput  },
  // - General status (readonly)
  { "ConfigVers",                "Config-Version (major.minor)",              0,     65535,       1,          VALUE_UNIT1(valueUnit_none),         0,            2,       reg_uint16,   1,     true   },
  { "ConfigVersPatch",           "Config-Version (patch)",                    0,       255,       1,          VALUE_UNIT1(valueUnit_none),         2,            1,       reg_uint8,    2,     true   },
  { "hwVers",                    "Hardware-Version (major.minor)",            0,     65535,       1,          VALUE_UNIT1(valueUnit_none),         3,            2,       reg_uint16,   3,     true   },
  { "hwVersPatch",               "Hardware-Version (patch)",                  0,       255,       1,          VALUE_UNIT1(valueUnit_none),         5,            1,       reg_uint8,    4,     true   },
  { "swVersMcu",                 "Software-Version MCU (major.minor)",        0,     65535,       1,          VALUE_UNIT1(valueUnit_none),         6,            2,       reg_uint16,   5,     true   },
  { "swVersPatchMcu",            "Software-Version MCU (patch)",              0,       255,       1,          VALUE_UNIT1(valueUnit_none),         8,            1,       reg_uint8,    6,     true   },
  { "swVersFpga",                "Software-Version FPGA (major.minor)",       0,     65535,       1,          VALUE_UNIT1(valueUnit_none),         9,            2,       reg_uint16,   7,     true   },
  { "swVersPatchFpga",           "Software-Version FPGA (patch)",             0,       255,       1,          VALUE_UNIT1(valueUnit_none),         11,           1,       reg_uint8,    8,     true   },
  { "blVersMcu",                 "Bootloader-Version MCU (major.minor)",      0,     65535,       1,          VALUE_UNIT1(valueUnit_none),         12,           2,       reg_uint16,   9,     true   },
  { "blVersPatchMcu",            "Bootloader-Version MCU (patch)",            0,       255,       1,          VALUE_UNIT1(valueUnit_none),         14,           1,       reg_uint8,    10,    true   },
  { "blVersFpga",                "Bootloader-Version FPGA (major.minor)",     0,     65535,       1,          VALUE_UNIT1(valueUnit_none),         15,           2,       reg_uint16,   11,    true   },
  { "blVersPatchFpga",           "Bootloader-Version FPGA (patch)",           0,       255,       1,          VALUE_UNIT1(valueUnit_none),         17,           1,       reg_uint8,    12,    true   },
  { "modulAddr",                 "Moduladresse (Drehschalter)",               0,       255,       1,          VALUE_UNIT1(valueUnit_none),         18,           1,       reg_uint8,    13,    true   },
  { "status0",                   "Zustand Ultraschallgenerierung",            0,       255,       1,          VALUE_UNIT1(valueUnit_none),         19,           1,       reg_uint8,    14,    true   },
  { "status1",                   "Betriebszustand",                           0,       255,       1,          VALUE_UNIT1(valueUnit_none),         20,           1,       reg_uint8,    15,    true   },
  { "error",                     "Anzeige Fehlerabschaltung",                 0,       255,       1,          VALUE_UNIT1(valueUnit_none),         21,           1,       reg_uint8,    16,    true   },
  { "warning",                   "Anzeige Warnung",                           0,       255,       1,          VALUE_UNIT1(valueUnit_none),         22,           1,       reg_uint8,    17,    true   },
  { "actualPower",               "Aktuelle Ist-Leistung",                     0,       100,       1,          VALUE_UNIT1(valueUnit_percent),      23,           1,       reg_uint8,    18,    true   },
  { "actualFrequency",           "Aktuelle Ist-Frequenz",                     0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        24,           2,       reg_uint16,   19,    true   },
  { "actualPhase",               "Ist-Phasenlage",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       26,           1,       reg_sint8,    20,    true   },
  { "temperaturQ1",              "Temperatur Schaltelement 1",                0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      27,           1,       reg_uint8,    21,    true   },
  { "temperaturQ2",              "Temperatur Schaltelement 2",                0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      28,           1,       reg_uint8,    22,    true   },
  { "temperaturQ3",              "Temperatur Schaltelement 3",                0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      29,           1,       reg_uint8,    23,    true   },
  { "temperaturQ4",              "Temperatur Schaltelement 4",                0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      30,           1,       reg_uint8,    24,    true   },
  { "temperaturPcb",             "Gehäuse Innentemperatur",                   0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      31,           1,       reg_uint8,    25,    true   },
  { "powerP",                    "Ist-Wirkleistung in Watt",                  1,       3000,      1,          VALUE_UNIT1(valueUnit_watt),         32,           2,       reg_uint16,   26,    true   },
  { "powerS",                    "Ist-Scheinleistung in Watt",                1,       3000,      1,          VALUE_UNIT1(valueUnit_voltampere),   34,           2,       reg_uint16,   27,    true   },
  { "current",                   "HF-Strom",                                  0,       255,       0.1,        VALUE_UNIT1(valueUnit_ampere),       36,           1,       reg_uint8,    28,    true   },
  { "voltagePowerStage",         "Spannung an Endstufe (Mittelwert)",         0,       255,       2,          VALUE_UNIT1(valueUnit_volt),         37,           1,       reg_uint8,    29,    true   },
  { "peakVoltagePowerStage",     "Spannung an Endstufe (Peak)",               0,       255,       2,          VALUE_UNIT1(valueUnit_volt),         38,           1,       reg_uint8,    30,    true   },
  { "pulsWidthPowerState",       "Stellwert Endstufe",                        0,       255,       1,          VALUE_UNIT1(valueUnit_percent),      39,           1,       reg_uint8,    31,    true   },
  { "serNr",                     "Serienummer Gerät",                         0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         40,           2,       reg_uint16,   32,    true   },

  // - General control (readwrite)
  { "control0",                  "Kontrollregister",                          0,       255,       1,          VALUE_UNIT1(valueUnit_none),         50,           1,       reg_uint8,    1,     false  },
  { "control1",                  "Kontrollregister",                          0,       255,       1,          VALUE_UNIT1(valueUnit_none),         51,           1,       reg_uint8,    2,     false  },
  { "targetPower",               "Sollleistung in %",                         10,      100,       1,          VALUE_UNIT1(valueUnit_percent),      52,           1,       reg_uint8,    3,     false  },
  { "targetPhase",               "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       53,           1,       reg_sint8,    4,     false  },
  { "frqMin",                    "Untere Grenze Frequenzband",                0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        54,           2,       reg_uint16,   5,     false  },
  { "frqMax",                    "Obere Grenze Frequenzband",                 0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        56,           2,       reg_uint16,   6,     false  },
  { "powerRange",                "Einstellung maximale Leistung",             0,       4000,      1,          VALUE_UNIT1(valueUnit_watt) ,        58,           2,       reg_uint16,   7,     false  },
  { "degasCycleTime",            "Degas Zykluszeit",                          0,       255,       1,          VALUE_UNIT1(valueUnit_none),         60,           1,       reg_uint8,    8,     false  },
  { "degasTime",                 "Degas Zeit",                                0,       255,       1,          VALUE_UNIT1(valueUnit_none),         61,           1,       reg_uint8,    9,     false  },
  { "degasCycleCount",           "Degas Zykluszähler",                        0,       255,       1,          VALUE_UNIT1(valueUnit_none),         62,           1,       reg_uint8,    10,    false  },
  { "fwOptions",                 "Firmware-Optionen",                         0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         63,           2,       reg_uint16,   11,    false  },
  { "customNr",                  "Kundenserienummer",                         0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         65,           2,       reg_uint16,   12,    false  },
  { "operatingTime",             "Betriebsdauer in Minuten",                  0,       16777215,  1,          VALUE_UNIT1(valueUnit_minute),       67,           3,       reg_uint24,   13,    false  },
  { "cntPowerUp",                "Powerup-Zähler",                            0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         70,           2,       reg_uint16,   16,    false  },
  { "cntCrash",                  "Absturzzähler",                             0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         72,           2,       reg_uint16,   17,    false  },


  // Frequenzband 1
  { "configSet1",                "Konfiguration zu Frequenzband 1",           0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         115,          2,       reg_uint16,   100,   false  },
  { "frqMinSet1",                "Untere Grenze Frequenzband 1",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        117,          2,       reg_uint16,   101,   false  },
  { "frqMaxSet1",                "Obere Grenze Frequenzband 1",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        119,          2,       reg_uint16,   102,   false  },
  { "phaseSet1",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       121,          1,       reg_sint8,    103,   false  },
  { "powerSet1",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      122,          1,       reg_uint8,    104,   false  },
  { "powerRangeSet1",            "Einstellung maximale Leistung",             0,       4000,      1,          VALUE_UNIT1(valueUnit_watt),         123,          2,       reg_uint16,   105,   false  },
  { "frqSweepShapeSet1",         "Kurvenform Wobbelung (frqSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         125,          1,       reg_uint8,    106,   false  },
  { "frqSweepModFrqSet1",        "Wobbelfrequenz (frqSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        126,          1,       reg_uint8,    107,   false  },
  { "frqSweepRangeSet1",         "Wobbelamplitude (frqSweep)",                0,       255,       100,        VALUE_UNIT1(valueUnit_hertz),        127,          1,       reg_uint8,    108,   false  },
  { "ampSweepShapeSet1",         "Kurvenform Wobbelung (ampSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         128,          1,       reg_uint8,    109,   false  },
  { "ampSweepFrqSet1",           "Wobbelfrequenz (ampSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        129,          1,       reg_uint8,    110,   false  },
  { "tempMaxQ1Set1",             "max. Temperatur Schaltelement 1",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      130,          1,       reg_uint8,    111,   false  },
  { "tempMaxQ2Set1",             "max. Temperatur Schaltelement 2",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      131,          1,       reg_uint8,    112,   false  },
  { "tempMaxQ3Set1",             "max. Temperatur Schaltelement 3",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      132,          1,       reg_uint8,    113,   false  },
  { "tempMaxQ4Set1",             "max. Temperatur Schaltelement 4",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      133,          1,       reg_uint8,    114,   false  },
  { "tempMaxPcbSet1",            "max. Temperatur PCB",                       0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      134,          1,       reg_uint8,    115,   false  },
  { "CntShortSet1",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         135,          2,       reg_uint16,   116,   false  },
  { "CntOverLoadSet1",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         137,          2,       reg_uint16,   117,   false  },
  { "CntOpenLoadSet1",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         139,          2,       reg_uint16,   118,   false  },
  { "CntOverVoltageSet1",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         141,          2,       reg_uint16,   119,   false  },
  { "CntOverTempSet1",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         143,          2,       reg_uint16,   120,   false  },
  { "CntNoFrqSet1",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         145,          2,       reg_uint16,   121,   false  },

  // Frequenzband 2
  { "configSet2",                "Konfiguration zu Frequenzband 2",           0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         150,          2,       reg_uint16,   130,   false  },
  { "frqMinSet2",                "Untere Grenze Frequenzband 2",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        152,          2,       reg_uint16,   131,   false  },
  { "frqMaxSet2",                "Obere Grenze Frequenzband 2",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        154,          2,       reg_uint16,   132,   false  },
  { "phaseSet2",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       156,          1,       reg_sint8,    133,   false  },
  { "powerSet2",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      157,          1,       reg_uint8,    134,   false  },
  { "powerRangeSet2",            "Einstellung maximale Leistung",             0,       4000,      1,          VALUE_UNIT1(valueUnit_watt),         158,          2,       reg_uint16,   135,   false  },
  { "frqSweepShapeSet2",         "Kurvenform Wobbelung (frqSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         160,          1,       reg_uint8,    136,   false  },
  { "frqSweepModFrqSet2",        "Wobbelfrequenz (frqSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        161,          1,       reg_uint8,    137,   false  },
  { "frqSweepRangeSet2",         "Wobbelamplitude (frqSweep)",                0,       255,       100,        VALUE_UNIT1(valueUnit_hertz),        162,          1,       reg_uint8,    138,   false  },
  { "ampSweepShapeSet2",         "Kurvenform Wobbelung (ampSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         163,          1,       reg_uint8,    139,   false  },
  { "ampSweepFrqSet2",           "Wobbelfrequenz (ampSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        164,          1,       reg_uint8,    140,   false  },
  { "tempMaxQ1Set2",             "max. Temperatur Schaltelement 1",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      165,          1,       reg_uint8,    141,   false  },
  { "tempMaxQ2Set2",             "max. Temperatur Schaltelement 2",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      166,          1,       reg_uint8,    142,   false  },
  { "tempMaxQ3Set2",             "max. Temperatur Schaltelement 3",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      167,          1,       reg_uint8,    143,   false  },
  { "tempMaxQ4Set2",             "max. Temperatur Schaltelement 4",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      168,          1,       reg_uint8,    144,   false  },
  { "tempMaxPcbSet2",            "max. Temperatur PCB",                       0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      169,          1,       reg_uint8,    145,   false  },
  { "CntShortSet2",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         170,          2,       reg_uint16,   146,   false  },
  { "CntOverLoadSet2",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         172,          2,       reg_uint16,   147,   false  },
  { "CntOpenLoadSet2",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         174,          2,       reg_uint16,   148,   false  },
  { "CntOverVoltageSet2",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         176,          2,       reg_uint16,   149,   false  },
  { "CntOverTempSet2",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         178,          2,       reg_uint16,   150,   false  },
  { "CntNoFrqSet2",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         180,          2,       reg_uint16,   151,   false  },

  // Frequenzband 3
  { "configSet3",                "Konfiguration zu Frequenzband 3",           0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         185,          2,       reg_uint16,   160,   false  },
  { "frqMinSet3",                "Untere Grenze Frequenzband 3",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        187,          2,       reg_uint16,   161,   false  },
  { "frqMaxSet3",                "Obere Grenze Frequenzband 3",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        189,          2,       reg_uint16,   162,   false  },
  { "phaseSet3",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       191,          1,       reg_sint8,    163,   false  },
  { "powerSet3",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      192,          1,       reg_uint8,    164,   false  },
  { "powerRangeSet3",            "Einstellung maximale Leistung",             0,       4000,      1,          VALUE_UNIT1(valueUnit_watt),         193,          2,       reg_uint16,   165,   false  },
  { "frqSweepShapeSet3",         "Kurvenform Wobbelung (frqSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         195,          1,       reg_uint8,    166,   false  },
  { "frqSweepModFrqSet3",        "Wobbelfrequenz (frqSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        196,          1,       reg_uint8,    167,   false  },
  { "frqSweepRangeSet3",         "Wobbelamplitude (frqSweep)",                0,       255,       100,        VALUE_UNIT1(valueUnit_hertz),        197,          1,       reg_uint8,    168,   false  },
  { "ampSweepShapeSet3",         "Kurvenform Wobbelung (ampSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         198,          1,       reg_uint8,    169,   false  },
  { "ampSweepFrqSet3",           "Wobbelfrequenz (ampSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        199,          1,       reg_uint8,    170,   false  },
  { "tempMaxQ1Set3",             "max. Temperatur Schaltelement 1",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      200,          1,       reg_uint8,    171,   false  },
  { "tempMaxQ2Set3",             "max. Temperatur Schaltelement 2",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      201,          1,       reg_uint8,    172,   false  },
  { "tempMaxQ3Set3",             "max. Temperatur Schaltelement 3",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      202,          1,       reg_uint8,    173,   false  },
  { "tempMaxQ4Set3",             "max. Temperatur Schaltelement 4",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      203,          1,       reg_uint8,    174,   false  },
  { "tempMaxPcbSet3",            "max. Temperatur PCB",                       0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      204,          1,       reg_uint8,    175,   false  },
  { "CntShortSet3",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         205,          2,       reg_uint16,   176,   false  },
  { "CntOverLoadSet3",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         207,          2,       reg_uint16,   177,   false  },
  { "CntOpenLoadSet3",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         209,          2,       reg_uint16,   178,   false  },
  { "CntOverVoltageSet3",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         211,          2,       reg_uint16,   179,   false  },
  { "CntOverTempSet3",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         213,          2,       reg_uint16,   180,   false  },
  { "CntNoFrqSet3",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         215,          2,       reg_uint16,   181,   false  },

  // Frequenzband 4
  { "configSet4",                "Konfiguration zu Frequenzband 4",           0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         220,          2,       reg_uint16,   190,   false  },
  { "frqMinSet4",                "Untere Grenze Frequenzband 4",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        222,          2,       reg_uint16,   191,   false  },
  { "frqMaxSet4",                "Obere Grenze Frequenzband 4",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        224,          2,       reg_uint16,   192,   false  },
  { "phaseSet4",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       226,          1,       reg_sint8,    193,   false  },
  { "powerSet4",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      227,          1,       reg_uint8,    194,   false  },
  { "powerRangeSet4",            "Einstellung maximale Leistung",             0,       4000,      1,          VALUE_UNIT1(valueUnit_watt),         228,          2,       reg_uint16,   195,   false  },
  { "frqSweepShapeSet4",         "Kurvenform Wobbelung (frqSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         230,          1,       reg_uint8,    196,   false  },
  { "frqSweepModFrqSet4",        "Wobbelfrequenz (frqSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        231,          1,       reg_uint8,    197,   false  },
  { "frqSweepRangeSet4",         "Wobbelamplitude (frqSweep)",                0,       255,       100,        VALUE_UNIT1(valueUnit_hertz),        232,          1,       reg_uint8,    198,   false  },
  { "ampSweepShapeSet4",         "Kurvenform Wobbelung (ampSweep)",           0,       3,         1,          VALUE_UNIT1(valueUnit_none),         233,          1,       reg_uint8,    199,   false  },
  { "ampSweepFrqSet4",           "Wobbelfrequenz (ampSweep)",                 0,       255,       1,          VALUE_UNIT1(valueUnit_hertz),        234,          1,       reg_uint8,    200,   false  },
  { "tempMaxQ1Set4",             "max. Temperatur Schaltelement 1",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      235,          1,       reg_uint8,    201,   false  },
  { "tempMaxQ2Set4",             "max. Temperatur Schaltelement 2",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      236,          1,       reg_uint8,    202,   false  },
  { "tempMaxQ3Set4",             "max. Temperatur Schaltelement 3",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      237,          1,       reg_uint8,    203,   false  },
  { "tempMaxQ4Set4",             "max. Temperatur Schaltelement 4",           0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      238,          1,       reg_uint8,    204,   false  },
  { "tempMaxPcbSet4",            "max. Temperatur PCB",                       0,       255,       0.5,        VALUE_UNIT1(valueUnit_celsius),      239,          1,       reg_uint8,    205,   false  },
  { "CntShortSet4",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         240,          2,       reg_uint16,   206,   false  },
  { "CntOverLoadSet4",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         242,          2,       reg_uint16,   207,   false  },
  { "CntOpenLoadSet4",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         244,          2,       reg_uint16,   208,   false  },
  { "CntOverVoltageSet4",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         246,          2,       reg_uint16,   209,   false  },
  { "CntOverTempSet4",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         248,          2,       reg_uint16,   210,   false  },
  { "CntNoFrqSet4",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         250,          2,       reg_uint16,   211,   false  },
};
static const int numModuleRegisters = sizeof(coreModuleRegisterDefs)/sizeof(CoreModuleRegister);


// MARK: - CoreRegModel

CoreRegModel::CoreRegModel()
{
}


CoreRegModel::~CoreRegModel()
{
}


CoreRegModel::RegIndex CoreRegModel::maxReg()
{
  return numModuleRegisters-1;
}




CoreRegModel::RegIndex CoreRegModel::regindexFromModbusReg(int aModbusReg, bool aInput)
{
  for (RegIndex i=0; i<numModuleRegisters; i++) {
    if (coreModuleRegisterDefs[i].mbreg==aModbusReg && coreModuleRegisterDefs[i].mbinput==aInput) {
      return i;
    }
  }
  return numModuleRegisters; // invalid index
}


CoreRegModel::RegIndex CoreRegModel::regindexFromRegName(const string aRegName)
{
  for (RegIndex i=0; i<numModuleRegisters; i++) {
    if (strucmp(coreModuleRegisterDefs[i].regname, aRegName.c_str())==0) {
      return i;
    }
  }
  return numModuleRegisters; // invalid index
}


ErrorPtr CoreRegModel::updateRegisterCache()
{
  // simply update all registers
  return updateRegisterCacheFromHardware(0, maxReg());
}


ErrorPtr CoreRegModel::checkUserInput(RegIndex aRegIdx, int32_t aValue)
{
  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  if (regP->mbinput) {
    return Error::err<CoreRegError>(CoreRegError::readOnly, "Register %s (index %d) is read-only", regP->regname, aRegIdx);
  }
  if (
    !(regP->max==0 && regP->min==0) && // min and max zero means no range limit
    (aValue>regP->max || aValue<regP->min)
  ) {
    return Error::err<CoreRegError>(CoreRegError::outOfRange, "Value is out of range for register %s (index %d)", regP->regname, aRegIdx);
  }
  return ErrorPtr();
}


ErrorPtr CoreRegModel::getUserValue(RegIndex aRegIdx, double& aValue)
{
  int32_t engval;
  ErrorPtr err = getEngineeringValue(aRegIdx, engval);
  if (Error::isOK(err)) {
    aValue = coreModuleRegisterDefs[aRegIdx].resolution*engval;
  }
  return err;
}


ErrorPtr CoreRegModel::setUserValue(RegIndex aRegIdx, double aValue)
{
  if (aRegIdx>=numModuleRegisters) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  return setEngineeringValue(aRegIdx, (int32_t)(aValue/coreModuleRegisterDefs[aRegIdx].resolution), true);
}


JsonObjectPtr CoreRegModel::getRegisterInfo(RegIndex aRegIdx)
{
  JsonObjectPtr info;
  if (aRegIdx<numModuleRegisters) {
    int32_t engval = 0;
    const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
    info = JsonObject::newObj();
    info->add("regidx", JsonObject::newInt32(aRegIdx));
    info->add("regname", JsonObject::newString(regP->regname));
    info->add("description", JsonObject::newString(regP->description));
    info->add("min", JsonObject::newDouble(regP->resolution*regP->min));
    info->add("max", JsonObject::newDouble(regP->resolution*regP->max));
    info->add("resolution", JsonObject::newDouble(regP->resolution));
    info->add("unit", JsonObject::newString(valueUnitName(regP->unit, false)));
    info->add("symbol", JsonObject::newString(valueUnitName(regP->unit, true)));
    info->add("spiaddr", JsonObject::newInt32(regP->addr));
    info->add("rawlen", JsonObject::newInt32(regP->rawlen));
    info->add("modbusreg", JsonObject::newInt32(regP->mbreg));
    info->add("readonly", JsonObject::newBool(regP->mbinput));
    ErrorPtr err = getEngineeringValue(aRegIdx, engval);
    if (Error::isOK(err)) {
      double val = regP->resolution*engval;
      info->add("engval", JsonObject::newInt32(engval));
      info->add("value", JsonObject::newDouble(val));
      int fracDigits = (int)(-::log(regP->resolution)/::log(10)+0.99);
      if (fracDigits<0) fracDigits=0;
      info->add("formatted", JsonObject::newString(string_format("%0.*f %s", fracDigits, val, valueUnitName(regP->unit, true).c_str())));
    }
    else {
      info->add("error", JsonObject::newString(err->text()));
      info->add("formatted", JsonObject::newString("<error>"));
    }
  }
  return info;
}


ErrorPtr CoreRegModel::setRegisterValue(RegIndex aRegIdx, JsonObjectPtr aNewValue)
{
  // for now, just convert to double and then set as user value
  // TODO: maybe enhance parsing for enums, flags etc.
  if (!aNewValue) {
    return Error::err<CoreRegError>(CoreRegError::invalidInput, "missing value");
  }
  string nvs = aNewValue->stringValue();
  double nvd;
  if (sscanf(nvs.c_str(), "%lf", &nvd)!=1) {
    return Error::err<CoreRegError>(CoreRegError::invalidInput, "invalid number");
  }
  return setUserValue(aRegIdx, nvd);
}


JsonObjectPtr CoreRegModel::getRegisterInfos()
{
  JsonObjectPtr infos = JsonObject::newArray();
  for (RegIndex i=0; i<numModuleRegisters; i++) {
    infos->arrayAppend(getRegisterInfo(i));
  }
  return infos;
}



// MARK: - SPICoreRegModel

SPICoreRegModel::SPICoreRegModel()
{
  // set up register model
  modbusSlave().setRegisterModel(
    0, 0,
    0, 0,
    mbreg_first, mb_numregs,
    mbinp_first, mb_numinps
  );
}


SPICoreRegModel::~SPICoreRegModel()
{
}


ModbusSlave& SPICoreRegModel::modbusSlave()
{
  if (!mModbusSlave) {
    mModbusSlave = new ModbusSlave();
  }
  return *mModbusSlave.get();
}


CoreSPIProto& SPICoreRegModel::coreSPIProto()
{
  if (!mCoreSPIProto) {
    mCoreSPIProto = new CoreSPIProto();
  }
  return *mCoreSPIProto.get();
}


ErrorPtr SPICoreRegModel::readSPIRegRange(RegIndex aFromIdx, RegIndex &aToIdx, uint8_t* aBuffer, size_t aBufSize)
{
  if (aFromIdx>=numModuleRegisters) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  const CoreModuleRegister* firstRegP = &coreModuleRegisterDefs[aFromIdx];
  const CoreModuleRegister* regP = firstRegP;
  size_t blksz = 0;
  RegIndex ridx = aFromIdx;
  // find contiguous block to read
  while (ridx<=aToIdx && regP->rawlen+blksz<=aBufSize) {
    blksz+=regP->rawlen;
    ridx++;
    if (ridx<=aToIdx && regP->addr+regP->rawlen!=(regP+1)->addr) {
      // next register not contiguous in SPI address space
      regP = NULL;
      break;
    }
    regP++;
  }
  // ridx now is the index+1 of the last register covered
  ErrorPtr err = coreSPIProto().readData(firstRegP->addr, blksz, aBuffer);
  if (Error::notOK(err)) {
    err->prefixMessage("Reading from register %s (index %d): ", firstRegP->regname, aFromIdx);
    return err;
  }
  aToIdx = ridx-1;
  return ErrorPtr();
}


static int32_t extractReg(const CoreModuleRegister* aRegP, const uint8_t* aDataP)
{
  int nb = aRegP->layout & reg_bytecount_mask;
  uint32_t data = 0;
  // LSB first
  for (int bi=0; bi<nb; bi++) {
    data = data + (*(aDataP+bi)<<8*bi);
  }
  // now we have the unsigned portion
  if (aRegP->layout & reg_signed && nb<4) {
    if (*(aDataP+nb-1) & 0x80) {
      data |= (0xFFFFFFFF<<nb*8); // extend sign bit
    }
  }
  return (int32_t)data;
}


ErrorPtr SPICoreRegModel::readRegFromBuffer(RegIndex aRegIdx, int32_t &aData, uint8_t* aBuffer, RegIndex aFirstRegIdx, RegIndex aLastRegIdx)
{
  if (aLastRegIdx>=numModuleRegisters || aRegIdx>aLastRegIdx || aRegIdx<aFirstRegIdx) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  const uint8_t* dataP = aBuffer + (regP->addr-coreModuleRegisterDefs[aFirstRegIdx].addr);
  aData = extractReg(regP, dataP);
  return ErrorPtr();
}


ErrorPtr SPICoreRegModel::readSPIReg(RegIndex aRegIdx, int32_t &aData)
{
  uint8_t buf[4];
  ErrorPtr err = readSPIRegRange(aRegIdx, aRegIdx, buf, 4);
  if (Error::isOK(err)) {
    err = readRegFromBuffer(aRegIdx, aData, buf, aRegIdx, aRegIdx);
  }
  return err;
}


static void layoutReg(const CoreModuleRegister* aRegP, const int32_t aData, uint8_t* aDataP)
{
  int nb = aRegP->layout & reg_bytecount_mask;
  uint32_t data = (uint32_t)aData;
  // LSB first
  for (int bi=0; bi<nb; bi++) {
    *aDataP++ = data & 0xFF;
    data >>= 8;
  }
}


ErrorPtr SPICoreRegModel::writeSPIReg(RegIndex aRegIdx, int32_t aData)
{
  if (aRegIdx>=numModuleRegisters) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  uint8_t buf[4];
  layoutReg(regP, aData, buf);
  ErrorPtr err = coreSPIProto().writeData(regP->addr, regP->rawlen, buf);
  if (Error::notOK(err)) {
    err->prefixMessage("Writing register %s (index %d): ", regP->regname, aRegIdx);
  }
  return err;
}


ErrorPtr SPICoreRegModel::updateRegisterCacheFromHardware(RegIndex aFromIdx, RegIndex aToIdx)
{
  ErrorPtr err;
  const size_t bufsz = numModuleRegisters*3; // enough for any range
  uint8_t buf[bufsz];
  while (aFromIdx<=aToIdx) {
    RegIndex t = aToIdx;
    err = readSPIRegRange(aFromIdx, t, buf, bufsz);
    if (Error::notOK(err)) return err;
    for (RegIndex i=aFromIdx; i<=t; i++) {
      int32_t data;
      err = readRegFromBuffer(i, data, buf, aFromIdx, t);
      if (Error::notOK(err)) return err;
      err = setEngineeringValue(i, data, false); // not user input, allow setting input registers and out-of-bounds values
    }
    aFromIdx = t+1;
  }
  return err;
}


ErrorPtr SPICoreRegModel::updateHardwareFromRegisterCache(RegIndex aRegIdx)
{
  int32_t data;
  ErrorPtr err = getEngineeringValue(aRegIdx, data);
  if (Error::isOK(err)) {
    err = writeSPIReg(aRegIdx, data);
  }
  return err;
}


ErrorPtr SPICoreRegModel::getEngineeringValue(RegIndex aRegIdx, int32_t& aValue)
{
  if (aRegIdx>=numModuleRegisters) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  uint32_t data = modbusSlave().getReg(regP->mbreg, regP->mbinput); // LSWord
  if ((regP->layout&reg_bytecount_mask)>2) {
    data |= (uint32_t)(modbusSlave().getReg(regP->mbreg+1, regP->mbinput))<<16; // MSWord
  }
  aValue = data;
  return ErrorPtr();
}


ErrorPtr SPICoreRegModel::setEngineeringValue(RegIndex aRegIdx, int32_t aValue, bool aUserInput)
{
  ErrorPtr err;

  if (aRegIdx>=numModuleRegisters) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  if (aUserInput) {
    err = checkUserInput(aRegIdx, aValue);
  }
  if (Error::isOK(err)) {
    const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
    modbusSlave().setReg(regP->mbreg, regP->mbinput, (uint16_t)aValue); // LSWord
    if ((regP->layout&reg_bytecount_mask)>2) {
      modbusSlave().setReg(regP->mbreg+1, regP->mbinput, (uint16_t)(aValue>>16)); // MSWord
    }
  }
  return err;
}

// MARK: - ProxyCoreRegModel


ProxyCoreRegModel::ProxyCoreRegModel()
{
  // initialize register value cache with all zeroes
  mRegisterValues.reserve(maxReg()+1);
  for (int i=0; i<=maxReg(); i++) {
    mRegisterValues.push_back(0);
  }
}


ProxyCoreRegModel::~ProxyCoreRegModel()
{
}


ModbusMaster& ProxyCoreRegModel::modbusMaster()
{
  if (!mModbusMaster) {
    mModbusMaster = new ModbusMaster();
  }
  return *mModbusMaster.get();
}


ErrorPtr ProxyCoreRegModel::getEngineeringValue(RegIndex aRegIdx, int32_t& aValue)
{
  if (aRegIdx>=numModuleRegisters || aRegIdx>=mRegisterValues.size()) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  if (aRegIdx>=mRegisterValues.size()) aValue = 0; // should not happen: return 0 for not yet cached values
  else aValue = mRegisterValues[aRegIdx];
  return ErrorPtr();
}


ErrorPtr ProxyCoreRegModel::setEngineeringValue(RegIndex aRegIdx, int32_t aValue, bool aUserInput)
{
  ErrorPtr err;

  if (aRegIdx>=numModuleRegisters || aRegIdx>=mRegisterValues.size()) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  if (aUserInput) {
    err = checkUserInput(aRegIdx, aValue);
  }
  if (Error::isOK(err)) {
    mRegisterValues[aRegIdx] = aValue;
  }
  return err;
}


ErrorPtr ProxyCoreRegModel::modbusReadRegisterSequence(RegIndex aFromIdx, int aNumModbusRegs, bool& aConnected)
{
  ErrorPtr err;
  uint16_t* regValuesP = new uint16_t[aNumModbusRegs]; // double size because some could be 32bit!
  const CoreModuleRegister* firstRegP = &coreModuleRegisterDefs[aFromIdx];
  if (!aConnected) {
    modbusMaster().connectAsMaster();
    aConnected = true;
  }
  DBGLOG(LOG_INFO, "modbusReadRegisterSequence: from %s(%d), mbreg=%d(%s), num_mbregs=%d", firstRegP->regname, aFromIdx, firstRegP->mbreg, firstRegP->mbinput ? "RO" : "RW", aNumModbusRegs);
  err = modbusMaster().readRegisters(firstRegP->mbreg, aNumModbusRegs, regValuesP, firstRegP->mbinput);
  RegIndex reg = aFromIdx; // must count these separately because there might be 32-bit registers (with 2 modbus regs)
  if (Error::isOK(err)) {
    // copy into our internal cache
    for (RegIndex i = 0; i<aNumModbusRegs; i++) {
      const CoreModuleRegister* regP = &coreModuleRegisterDefs[aFromIdx+i];
      int32_t val;
      if ((regP->layout&reg_bytecount_mask)>2) {
        // 32bit: first 16bit are LSWord, second 16bit are MSWord
        val = (int32_t)((uint32_t)regValuesP[i] + ((uint32_t)regValuesP[i+1]<<16)); // sign comes from highest bit MSWord
        i++;
      }
      else if (regP->layout&reg_signed) {
        // 16bit signed
        val = (int16_t)regValuesP[i]; // sign will be extended to 32bit
      }
      else {
        // 16bit unsigned
        val = (uint16_t)regValuesP[i]; // no sign extension!
      }
      setEngineeringValue(reg, val, false);
      reg++;
    }
  }
  delete[] regValuesP;
  return err;
}




ErrorPtr ProxyCoreRegModel::updateRegisterCacheFromHardware(RegIndex aFromIdx, RegIndex aToIdx)
{
  ErrorPtr err;

  uint16_t last_mbreg; // none
  RegIndex seqstart; // first register index for a multi-register modbus read call
  bool inp_mbregs; // is current sequence an input register?
  uint16_t num_mbregs = 0; // number of registers accumulated
  bool connected = modbusMaster().isConnected(); // same modbus connection for all needed register calls

  for (RegIndex reg = aFromIdx; reg<=aToIdx; reg++) {
    const CoreModuleRegister* regP = &coreModuleRegisterDefs[reg];
    if (num_mbregs>0) {
      // possible continuation of sequence
      if (regP->mbinput==inp_mbregs && regP->mbreg==last_mbreg+1) {
        // extend sequence
        num_mbregs++;
        if ((regP->layout&reg_bytecount_mask)>2) {
          // read next modbus register (MSWord) as well when value is >16bit
          num_mbregs++;
          last_mbreg++; // sequence must check that
        }
      }
      else {
        // previous sequence ends, actually get values via modbus
        err = modbusReadRegisterSequence(seqstart, num_mbregs, connected);
        num_mbregs = 0; // reset
        if (Error::notOK(err)) break;
      }
    }
    if (num_mbregs==0) {
      // start of new sequence
      seqstart = reg;
      inp_mbregs = regP->mbinput;
      num_mbregs++;
    }
    last_mbreg = regP->mbreg;
  }
  if (Error::isOK(err) && num_mbregs>0) {
    err = modbusReadRegisterSequence(seqstart, num_mbregs, connected);
  }

  if (!connected || Error::notOK(err)) {
    modbusMaster().close();
  }
  
  return err;
}


ErrorPtr ProxyCoreRegModel::updateHardwareFromRegisterCache(RegIndex aRegIdx)
{
  ErrorPtr err;

  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  if (regP->mbinput) return Error::err<CoreRegError>(CoreRegError::readOnly, "cannot update read-only modbus input #%d", regP->mbreg);
  int32_t val;
  err = getEngineeringValue(aRegIdx, val);
  if (Error::isOK(err)) {
    uint16_t regs[2];
    regs[0] = val & 0xFFFF; // LSWord
    if ((regP->layout&reg_bytecount_mask)>2) {
      // 32bit value, must send it as two consecutive registers
      regs[1] = (uint32_t)val >> 16;
      err = modbusMaster().writeRegisters(regP->mbreg, 2, regs);
    }
    else {
      // 16bit value, just LSWord alone
      err = modbusMaster().writeRegisters(regP->mbreg, 1, regs);
    }
  }
  return err;
}



