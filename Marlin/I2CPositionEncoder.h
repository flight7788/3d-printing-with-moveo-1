/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016, 2017 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef I2CPOSENC_H
#define I2CPOSENC_H

#include "MarlinConfig.h"

#if ENABLED(I2C_POSITION_ENCODERS)

  #include "enum.h"
  #include "macros.h"
  #include "types.h"
  #include "planner.h"
  #include "I2C.h"

  

class I2CPositionEncodersMgr {
  private:
    uint8_t addr,cmd;
    uint8_t buffer[ENCODER_BUF_SIZE];
    millis_t lastPositionTime;
    float lastposition_joint[Joint_All];
    int32_t lastposition_joint_steps[Joint_All];
    const uint32_t Threshold_Speed_per_steps[Joint_All] = Threshold_Speed;

  public:
    bool ConstECHO_f, ConstUpdate_f, PlannerECHO_f, PrintStatue_f, ReadStatus, ErrorSteps_f, diff_f, Manual_f, Update_f, Speed_f;
    float position_joint_SAD[Joint_All];
    float position_joint[Joint_All], Current_joint[Joint_All], Current_speed[Joint_All];;
    int32_t position_joint_steps[Joint_All], Current_joint_steps[Joint_All], Current_deltadistance[Joint_All];
    millis_t DeltaTime;
    //int32_t Record_Command_joint_steps[30][Joint_All], Record_Current_joint_steps[30][Joint_All];

    void init(void);

    void reset(void);

    void update(void);

    uint8_t get_raw_count(float (&joint)[Joint_All]);

    void get_joint_steps(float (&joint_degree)[Joint_All] ,int32_t (&joint_steps)[Joint_All]);

    /*
    static void report_position(const int8_t idx, const bool units, const bool noOffset);
    
    static void report_status(const int8_t idx) {
      CHECK_IDX();
      SERIAL_ECHOPAIR("Encoder ",idx);
      SERIAL_ECHOPGM(": ");
      encoders[idx].get_raw_count();
      encoders[idx].passes_test(true);
    }

    

    static int8_t parse();

    
    static void M861();
    static void M862();
    static void M863();
    static void M864();
    static void M865();
    
    static void M867();
    static void M868();
    static void M869();
    */
    //void M861();
    void M860();
    void M866();
  };

  extern I2CPositionEncodersMgr I2CPEM;
  extern Planner planner;

  FORCE_INLINE void gcode_M860() { I2CPEM.M860(); }
  FORCE_INLINE void gcode_M861() { ;}//I2CPEM.M861(); }
  FORCE_INLINE void gcode_M862() { ;}//I2CPEM.M862(); }
  FORCE_INLINE void gcode_M863() { ;}//I2CPEM.M863(); }
  FORCE_INLINE void gcode_M864() { ;}//I2CPEM.M864(); }
  FORCE_INLINE void gcode_M865() { ;}//I2CPEM.M865(); }
  FORCE_INLINE void gcode_M866() { I2CPEM.M866(); }
  FORCE_INLINE void gcode_M867() { ;}//I2CPEM.M867(); }
  FORCE_INLINE void gcode_M868() { ;}//I2CPEM.M868(); }
  FORCE_INLINE void gcode_M869() { ;}//I2CPEM.M869(); }

#endif //I2C_POSITION_ENCODERS
#endif //I2CPOSENC_H
