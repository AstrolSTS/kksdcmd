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
const int mb_numinps = 250-mbreg_first+1;

// Core module register definitions
const CoreModuleRegister coreModuleRegisterDefs[] = {
  // regname                      description                                 min,     max,       resolution, unit,                                addr,         rawlen,  layout,       mbreg, mbinput  },
  // - General status (readonly)
  { "modulAddr",                 "Moduladresse (Drehschalter)",               0,       255,       1,          VALUE_UNIT1(valueUnit_none),         0,            1,       reg_uint8,    1,     true   },
  { "status0",                   "Zustand Ultraschallgenerierung",            0,       0,         1,          VALUE_UNIT1(valueUnit_none),         1,            1,       reg_uint8,    2,     true   },
  { "status1",                   "Betriebszustand",                           0,       0,         1,          VALUE_UNIT1(valueUnit_none),         2,            1,       reg_uint8,    3,     true   },
  { "error",                     "Anzeige Fehlerzustand",                     0,       0,         1,          VALUE_UNIT1(valueUnit_none),         3,            2,       reg_uint16,   4,     true   },
  { "actualPower",               "Aktuelle Ist-Leistung",                     0,       100,       1,          VALUE_UNIT1(valueUnit_percent),      5,            1,       reg_uint8,    5,     true   },
  { "actualFrequency",           "Aktuelle Ist-Frequenz",                     0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        6,            2,       reg_uint16,   6,     true   },
  { "actualPhase",               "Ist-Phasenlage",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       8,            1,       reg_sint8,    7,     true   },
  { "temperaturQ1",              "Temperatur Schaltelement 1",                -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      9,            1,       reg_sint8,    8,     true   },
  { "temperaturQ2",              "Temperatur Schaltelement 2",                -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      10,           1,       reg_sint8,    9,     true   },
  { "temperaturQ3",              "Temperatur Schaltelement 3",                -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      11,           1,       reg_sint8,    10,    true   },
  { "temperaturQ4",              "Temperatur Schaltelement 4",                -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      12,           1,       reg_sint8,    11,    true   },
  { "temperaturPcb",             "Gehäuse Innentemperatur",                   -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      13,           1,       reg_sint8,    12,    true   },
  { "powerP",                    "Ist-Wirkleistung in Watt",                  1,       3000,      1,          VALUE_UNIT1(valueUnit_watt),         14,           2,       reg_uint16,   13,    true   },
  { "powerS",                    "Ist-Scheinleistung in Watt",                1,       3000,      1,          VALUE_UNIT1(valueUnit_voltampere),   16,           2,       reg_uint16,   14,    true   },
  { "current",                   "HF-Strom",                                  0,       255,       0.1,        VALUE_UNIT1(valueUnit_ampere),       18,           1,       reg_uint8,    15,    true   },
  { "voltagePowerStage",         "Spannung an Endstufe (Mittelwert)",         0,       255,       2,          VALUE_UNIT1(valueUnit_volt),         19,           1,       reg_uint8,    16,    true   },
  { "peakVoltagePowerStage",     "Spannung an Endstufe (Peak)",               0,       255,       2,          VALUE_UNIT1(valueUnit_volt),         20,           1,       reg_uint8,    17,    true   },
  { "controllValuePowerState",   "Stellwert Endstufe",                        0,       0,         1,          VALUE_UNIT1(valueUnit_none),         21,           1,       reg_uint8,    18,    true   },
  { "serNr",                     "Serienummer Gerät",                         0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         22,           2,       reg_uint16,   19,    true   },
  { "customNr",                  "Kundenserienummer",                         0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         24,           2,       reg_uint16,   20,    true   },
  { "swVersMcu_major",           "Softwareversion MCU (Hauptversion)",        0,       255,       1,          VALUE_UNIT1(valueUnit_none),         26,           1,       reg_uint8,    21,    true   },
  { "swVersMcu_minor",           "Softwareversion MCU (Nebenversion)",        0,       255,       1,          VALUE_UNIT1(valueUnit_none),         27,           1,       reg_uint8,    22,    true   },
  { "swVersMcu_patch",           "Softwareversion MCU (Revision)",            0,       255,       1,          VALUE_UNIT1(valueUnit_none),         28,           1,       reg_uint8,    23,    true   },
  { "swVersFpga_patch",          "Softwareversion MCU (Hauptversion)",        0,       255,       1,          VALUE_UNIT1(valueUnit_none),         29,           1,       reg_uint8,    24,    true   },
  { "swVersFpga_minor",          "Softwareversion MCU (Nebenversion)",        0,       255,       1,          VALUE_UNIT1(valueUnit_none),         30,           1,       reg_uint8,    25,    true   },
  { "swVersFpga_major",          "Softwareversion MCU (Revision)",            0,       255,       1,          VALUE_UNIT1(valueUnit_none),         31,           1,       reg_uint8,    26,    true   },
  { "operatingTime",             "Betriebsdauer in Minuten",                  0,       16777215,  1,          VALUE_UNIT1(valueUnit_minute),       32,           3,       reg_uint24,   27,    true   },
  { "cntPowerUp",                "Powerup-Zähler",                            0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         35,           2,       reg_uint16,   29,    true   },
  { "cntCrash",                  "Absturzzähler",                             0,       65535,     1,          VALUE_UNIT1(valueUnit_none),         37,           2,       reg_uint16,   30,    true   },
  // - General control (readwrite)
  { "control0",                  "Kontrollregister",                          0,       0,         1,          VALUE_UNIT1(valueUnit_none),         50,           1,       reg_uint8,    1,     false  },
  { "control1",                  "Kontrollregister",                          0,       0,         1,          VALUE_UNIT1(valueUnit_none),         51,           1,       reg_uint8,    2,     false  },
  { "setPower",                  "Sollleistung in %",                         10,      100,       1,          VALUE_UNIT1(valueUnit_percent),      52,           1,       reg_uint8,    3,     false  },
  { "setPhase",                  "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       53,           1,       reg_sint8,    4,     false  },
  { "frqMin",                    "Untere Grenze Frequenzband",                0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        54,           2,       reg_uint16,   5,     false  },
  { "frqMax",                    "Obere Grenze Frequenzband",                 0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        56,           2,       reg_uint16,   6,     false  },
  { "powerRange",                "Leistung bei 100% in % der max. Leistung",  0,       255,       0.5,        VALUE_UNIT1(valueUnit_percent) ,     58,           1,       reg_uint8,    7,     false  },
  { "degasCycleTime",            "?",                                         0,       255,       1,          VALUE_UNIT1(unit_unknown),           59,           1,       reg_uint8,    8,     false  },
  { "degasTime",                 "?",                                         0,       255,       1,          VALUE_UNIT1(unit_unknown),           60,           1,       reg_uint8,    9,     false  },
  { "degasCycleCount",           "?",                                         0,       255,       1,          VALUE_UNIT1(unit_unknown),           61,           1,       reg_uint8,    10,    false  },
  { "options",                   "Firmware-Optionen",                         0,       0,         1,          VALUE_UNIT1(valueUnit_none),         62,           2,       reg_uint16,   11,    false  },

  // Frequenzband 1
  // - control
  { "configSet1",                "Konfiguration zu Frequenzband 1",           0,       0,         1,          VALUE_UNIT1(valueUnit_none),         115,          2,       reg_uint16,   100,   false  },
  { "frqMinSet1",                "Untere Grenze Frequenzband 1",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        117,          2,       reg_uint16,   101,   false  },
  { "frqMaxSet1",                "Obere Grenze Frequenzband 1",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        119,          2,       reg_uint16,   102,   false  },
  { "initFrqSet1",               "Initialfrequenz Frequenzband 1",            0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        121,          2,       reg_uint16,   103,   false  },
  { "phaseSet1",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       123,          1,       reg_sint8,    104,   false  },
  { "powerSet1",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      124,          1,       reg_uint8,    105,   false  },
  { "powerRangeSet1",            "Leistung bei 100% in % der max. Leistung",  0,       255,       0.5,        VALUE_UNIT1(valueUnit_percent),      125,          1,       reg_uint8,    106,   false  },
  { "WobShapeSet1",              "Kurvenform Wobbelung",                      0,       3,         1,          VALUE_UNIT1(valueUnit_none),         126,          1,       reg_uint8,    107,   false  },
  { "WobFrqSet1",                "Wobbelfrequenz",                            0,       255,       1,          VALUE_UNIT1(unit_unknown),           127,          1,       reg_uint8,    108,   false  },
  { "WobAmplitudeSet1",          "Wobbelamplitude",                           0,       255,       1,          VALUE_UNIT1(unit_unknown),           128,          1,       reg_uint8,    109,   false  },
  // - readonly
  { "tempMaxQ1Set1",             "max. Temperatur Schaltelement 1",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      129,          1,       reg_sint8,    100,   true   },
  { "tempMaxQ2Set1",             "max. Temperatur Schaltelement 2",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      130,          1,       reg_sint8,    101,   true   },
  { "tempMaxQ3Set1",             "max. Temperatur Schaltelement 3",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      131,          1,       reg_sint8,    102,   true   },
  { "tempMaxQ4Set1",             "max. Temperatur Schaltelement 4",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      132,          1,       reg_sint8,    103,   true   },
  { "tempMaxPcbSet1",            "max. Temperatur PCB",                       -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      133,          1,       reg_sint8,    104,   true   },
  { "CntShortSet1",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         134,          2,       reg_uint16,   105,   true   },
  { "CntOverLoadSet1",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         136,          2,       reg_uint16,   106,   true   },
  { "CntOpenLoadSet1",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         138,          2,       reg_uint16,   107,   true   },
  { "CntOverVoltageSet1",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         140,          2,       reg_uint16,   108,   true   },
  { "CntOverTempSet1",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         142,          2,       reg_uint16,   109,   true   },
  { "CntNoFrqSet1",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         144,          2,       reg_uint16,   110,   true   },

  // Frequenzband 2
  // - control
  { "configSet2",                "Konfiguration zu Frequenzband 2",           0,       0,         1,          VALUE_UNIT1(valueUnit_none),         150,          2,       reg_uint16,   120,   false  },
  { "frqMinSet2",                "Untere Grenze Frequenzband 2",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        152,          2,       reg_uint16,   121,   false  },
  { "frqMaxSet2",                "Obere Grenze Frequenzband 2",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        154,          2,       reg_uint16,   122,   false  },
  { "initFrqSet2",               "Initialfrequenz Frequenzband 2",            0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        156,          2,       reg_uint16,   123,   false  },
  { "phaseSet2",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       158,          1,       reg_sint8,    124,   false  },
  { "powerSet2",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      159,          1,       reg_uint8,    125,   false  },
  { "powerRangeSet2",            "Leistung bei 100% in % der max. Leistung",  0,       255,       0.5,        VALUE_UNIT1(valueUnit_percent),      160,          1,       reg_uint8,    126,   false  },
  { "WobShapeSet2",              "Kurvenform Wobbelung",                      0,       3,         1,          VALUE_UNIT1(valueUnit_none),         161,          1,       reg_uint8,    127,   false  },
  { "WobFrqSet2",                "Wobbelfrequenz",                            0,       255,       1,          VALUE_UNIT1(unit_unknown),           162,          1,       reg_uint8,    128,   false  },
  { "WobAmplitudeSet2",          "Wobbelamplitude",                           0,       255,       1,          VALUE_UNIT1(unit_unknown),           163,          1,       reg_uint8,    129,   false  },
  // - readonly
  { "tempMaxQ1Set2",             "max. Temperatur Schaltelement 1",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      164,          1,       reg_sint8,    120,   true   },
  { "tempMaxQ2Set2",             "max. Temperatur Schaltelement 2",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      165,          1,       reg_sint8,    121,   true   },
  { "tempMaxQ3Set2",             "max. Temperatur Schaltelement 3",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      166,          1,       reg_sint8,    122,   true   },
  { "tempMaxQ4Set2",             "max. Temperatur Schaltelement 4",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      167,          1,       reg_sint8,    123,   true   },
  { "tempMaxPcbSet2",            "max. Temperatur PCB",                       -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      168,          1,       reg_sint8,    124,   true   },
  { "CntShortSet2",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         169,          2,       reg_uint16,   125,   true   },
  { "CntOverLoadSet2",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         171,          2,       reg_uint16,   126,   true   },
  { "CntOpenLoadSet2",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         173,          2,       reg_uint16,   127,   true   },
  { "CntOverVoltageSet2",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         175,          2,       reg_uint16,   128,   true   },
  { "CntOverTempSet2",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         177,          2,       reg_uint16,   129,   true   },
  { "CntNoFrqSet2",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         179,          2,       reg_uint16,   130,   true   },

  // Frequenzband 3
  // - control
  { "configSet3",                "Konfiguration zu Frequenzband 3",           0,       0,         1,          VALUE_UNIT1(valueUnit_none),         185,          2,       reg_uint16,   140,   false  },
  { "frqMinSet3",                "Untere Grenze Frequenzband 3",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        187,          2,       reg_uint16,   141,   false  },
  { "frqMaxSet3",                "Obere Grenze Frequenzband 3",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        189,          2,       reg_uint16,   142,   false  },
  { "initFrqSet3",               "Initialfrequenz Frequenzband 3",            0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        191,          2,       reg_uint16,   143,   false  },
  { "phaseSet3",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       193,          1,       reg_sint8,    144,   false  },
  { "powerSet3",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      194,          1,       reg_uint8,    145,   false  },
  { "powerRangeSet3",            "Leistung bei 100% in % der max. Leistung",  0,       255,       0.5,        VALUE_UNIT1(valueUnit_percent),      195,          1,       reg_uint8,    146,   false  },
  { "WobShapeSet3",              "Kurvenform Wobbelung",                      0,       3,         1,          VALUE_UNIT1(valueUnit_none),         196,          1,       reg_uint8,    147,   false  },
  { "WobFrqSet3",                "Wobbelfrequenz",                            0,       255,       1,          VALUE_UNIT1(unit_unknown),           197,          1,       reg_uint8,    148,   false  },
  { "WobAmplitudeSet3",          "Wobbelamplitude",                           0,       255,       1,          VALUE_UNIT1(unit_unknown),           198,          1,       reg_uint8,    149,   false  },
  // - readonly
  { "tempMaxQ1Set3",             "max. Temperatur Schaltelement 1",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      199,          1,       reg_sint8,    140,   true   },
  { "tempMaxQ2Set3",             "max. Temperatur Schaltelement 2",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      200,          1,       reg_sint8,    141,   true   },
  { "tempMaxQ3Set3",             "max. Temperatur Schaltelement 3",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      201,          1,       reg_sint8,    142,   true   },
  { "tempMaxQ4Set3",             "max. Temperatur Schaltelement 4",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      202,          1,       reg_sint8,    143,   true   },
  { "tempMaxPcbSet3",            "max. Temperatur PCB",                       -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      203,          1,       reg_sint8,    144,   true   },
  { "CntShortSet3",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         204,          2,       reg_uint16,   145,   true   },
  { "CntOverLoadSet3",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         206,          2,       reg_uint16,   146,   true   },
  { "CntOpenLoadSet3",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         208,          2,       reg_uint16,   147,   true   },
  { "CntOverVoltageSet3",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         210,          2,       reg_uint16,   148,   true   },
  { "CntOverTempSet3",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         212,          2,       reg_uint16,   149,   true   },
  { "CntNoFrqSet3",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         214,          2,       reg_uint16,   150,   true   },

  // Frequenzband 4
  // - control
  { "configSet4",                "Konfiguration zu Frequenzband 4",           0,       0,         1,          VALUE_UNIT1(valueUnit_none),         220,          2,       reg_uint16,   160,   false  },
  { "frqMinSet4",                "Untere Grenze Frequenzband 4",              0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        222,          2,       reg_uint16,   161,   false  },
  { "frqMaxSet4",                "Obere Grenze Frequenzband 4",               0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        224,          2,       reg_uint16,   162,   false  },
  { "initFrqSet4",               "Initialfrequenz Frequenzband 4",            0,       4000,      100,        VALUE_UNIT1(valueUnit_hertz),        226,          2,       reg_uint16,   163,   false  },
  { "phaseSet4",                 "Sollphase in °",                            -90,     90,        1,          VALUE_UNIT1(valueUnit_degree),       228,          1,       reg_sint8,    164,   false  },
  { "powerSet4",                 "Startwert Sollleistung in %",               1,       100,       1,          VALUE_UNIT1(valueUnit_percent),      229,          1,       reg_uint8,    165,   false  },
  { "powerRangeSet4",            "Leistung bei 100% in % der max. Leistung",  0,       255,       0.5,        VALUE_UNIT1(valueUnit_percent),      230,          1,       reg_uint8,    166,   false  },
  { "WobShapeSet4",              "Kurvenform Wobbelung",                      0,       3,         1,          VALUE_UNIT1(valueUnit_none),         231,          1,       reg_uint8,    167,   false  },
  { "WobFrqSet4",                "Wobbelfrequenz",                            0,       255,       1,          VALUE_UNIT1(unit_unknown),           232,          1,       reg_uint8,    168,   false  },
  { "WobAmplitudeSet4",          "Wobbelamplitude",                           0,       255,       1,          VALUE_UNIT1(unit_unknown),           233,          1,       reg_uint8,    169,   false  },
  // - readonly
  { "tempMaxQ1Set4",             "max. Temperatur Schaltelement 1",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      234,          1,       reg_sint8,    160,   true   },
  { "tempMaxQ2Set4",             "max. Temperatur Schaltelement 2",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      235,          1,       reg_sint8,    161,   true   },
  { "tempMaxQ3Set4",             "max. Temperatur Schaltelement 3",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      236,          1,       reg_sint8,    162,   true   },
  { "tempMaxQ4Set4",             "max. Temperatur Schaltelement 4",           -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      237,          1,       reg_sint8,    163,   true   },
  { "tempMaxPcbSet4",            "max. Temperatur PCB",                       -20,     127,       1,          VALUE_UNIT1(valueUnit_celsius),      238,          1,       reg_sint8,    164,   true   },
  { "CntShortSet4",              "Zähler Kurzschlussabschaltungen",           0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         239,          2,       reg_uint16,   165,   true   },
  { "CntOverLoadSet4",           "Zähler Überlastabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         241,          2,       reg_uint16,   166,   true   },
  { "CntOpenLoadSet4",           "Zähler Leerlaufabschaltungen",              0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         243,          2,       reg_uint16,   167,   true   },
  { "CntOverVoltageSet4",        "Zähler Überspannung",                       0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         245,          2,       reg_uint16,   168,   true   },
  { "CntOverTempSet4",           "Zähler Übertemperatur",                     0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         247,          2,       reg_uint16,   169,   true   },
  { "CntNoFrqSet4",              "Zähler kein Frequenzpunkt",                 0,       65335,     1,          VALUE_UNIT1(valueUnit_none),         249,          2,       reg_uint16,   170,   true   },
};
static const int numModuleRegisters = sizeof(coreModuleRegisterDefs)/sizeof(CoreModuleRegister);


CoreRegModel::CoreRegModel()
{
  // set up register model
  modbusSlave().setRegisterModel(
    0, 0,
    0, 0,
    mbreg_first, mb_numregs,
    mbinp_first, mb_numinps
  );
}


CoreRegModel::~CoreRegModel()
{
}


ModbusSlave& CoreRegModel::modbusSlave()
{
  if (!mModbusSlave) {
    mModbusSlave = new ModbusSlave();
  }
  return *mModbusSlave.get();
}


CoreSPIProto& CoreRegModel::coreSPIProto()
{
  if (!mCoreSPIProto) {
    mCoreSPIProto = new CoreSPIProto();
  }
  return *mCoreSPIProto.get();
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



ErrorPtr CoreRegModel::readSPIRegRange(RegIndex aFromIdx, RegIndex &aToIdx, uint8_t* aBuffer, size_t aBufSize)
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
    data = (data<<8) + *(aDataP+bi);
  }
  // now we have the unsigned portion
  if (aRegP->layout & reg_signed && nb<4) {
    if (*(aDataP+nb-1) & 0x80) {
      data |= (0xFFFFFFFF<<nb*8); // extend sign bit
    }
  }
  return (int32_t)data;
}



ErrorPtr CoreRegModel::readRegFromBuffer(RegIndex aRegIdx, int32_t &aData, uint8_t* aBuffer, RegIndex aFirstRegIdx, RegIndex aLastRegIdx)
{
  if (aLastRegIdx>=numModuleRegisters || aRegIdx>aLastRegIdx || aRegIdx<aFirstRegIdx) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  const uint8_t* dataP = aBuffer + (regP->addr-coreModuleRegisterDefs[aFirstRegIdx].addr);
  aData = extractReg(regP, dataP);
  return ErrorPtr();
}


ErrorPtr CoreRegModel::readSPIReg(RegIndex aRegIdx, int32_t &aData)
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


ErrorPtr CoreRegModel::writeSPIReg(RegIndex aRegIdx, int32_t aData)
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



ErrorPtr CoreRegModel::updateModbusRegistersFromSPI(RegIndex aFromIdx, RegIndex aToIdx)
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


ErrorPtr CoreRegModel::updateSPIRegisterFromModbus(RegIndex aRegIdx)
{
  int32_t data;
  ErrorPtr err = getEngineeringValue(aRegIdx, data);
  if (Error::isOK(err)) {
    err = writeSPIReg(aRegIdx, data);
  }
  return err;
}


ErrorPtr CoreRegModel::getEngineeringValue(RegIndex aRegIdx, int32_t& aValue)
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


ErrorPtr CoreRegModel::setEngineeringValue(RegIndex aRegIdx, int32_t aValue, bool aUserInput)
{
  if (aRegIdx>=numModuleRegisters) {
    return Error::err<CoreRegError>(CoreRegError::invalidIndex);
  }
  const CoreModuleRegister* regP = &coreModuleRegisterDefs[aRegIdx];
  if (aUserInput) {
    if (regP->mbinput) {
      return Error::err<CoreRegError>(CoreRegError::readOnly, "Register %s (index %d) is read-only", regP->regname, aRegIdx);
    }
    if (
      !(regP->max==0 && regP->min==0) && // min and max zero means no range limit
      (aValue>regP->max || aValue<regP->min)
    ) {
      return Error::err<CoreRegError>(CoreRegError::outOfRange, "Value is out of range for register %s (index %d)", regP->regname, aRegIdx);
    }
  }
  modbusSlave().setReg(regP->mbreg, regP->mbinput, (uint16_t)aValue); // LSWord
  if ((regP->layout&reg_bytecount_mask)>2) {
    modbusSlave().setReg(regP->mbreg+1, regP->mbinput, (uint16_t)(aValue>>16)); // MSWord
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
