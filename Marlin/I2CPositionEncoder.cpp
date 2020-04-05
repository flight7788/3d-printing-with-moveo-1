#include "MarlinConfig.h"

#if ENABLED(I2C_POSITION_ENCODERS)

  #include "Marlin.h"
  #include "temperature.h"
  #include "stepper.h"
  #include "I2CPositionEncoder.h"
  #include "parser.h"
  #include "I2C.h"


  void I2CPositionEncodersMgr::init() {
    addr = ENCODER_ADDR;
    cmd = ENCODER_CMD;
    I2c.begin();
    I2c.timeOut(10);
    SERIAL_ECHOLNPGM("Setting up encoder ... ");
    SERIAL_ECHOLNPAIR("Joint encoder, addr = ", addr);
    update();
    LOOP_NUM_JOINT(i){
      position_joint_SAD[i] = 0;
    }
  }

  void I2CPositionEncodersMgr::reset(){
    SERIAL_ECHOLNPGM("Resetting encoder ...");
    update();
    LOOP_NUM_JOINT(i){
      position_joint_SAD[i] = 0;
    }
  }

  void I2CPositionEncodersMgr::update() {
    switch(get_raw_count(position_joint)){
      case 0: break;
      case 1: SERIAL_ECHOLNPGM("Function timed out waiting for successful completion of a Start bit");                  break;
      case 2: SERIAL_ECHOLNPGM("Function timed out waiting for ACK/NACK while addressing slave in transmit mode (MT)"); break;
      case 3: SERIAL_ECHOLNPGM("Function timed out waiting for ACK/NACK while sending data to the slave");              break;
      case 4: SERIAL_ECHOLNPGM("Function timed out waiting for successful completion of a Repeated Start");             break;
      case 5: SERIAL_ECHOLNPGM("Function timed out waiting for ACK/NACK while addressing slave in receiver mode (MR)"); break;
      case 6: SERIAL_ECHOLNPGM("Function timed out waiting for ACK/NACK while receiving data from the slave");          break;
      case 7: SERIAL_ECHOLNPGM("Function timed out waiting for successful completion of the Stop bit");                 break;
      default: SERIAL_ECHOLNPGM("See datasheet for exact meaning"); break;
    }
  }

  uint8_t I2CPositionEncodersMgr::get_raw_count(float (&joint)[Joint_All]) {
    uint8_t status = I2c.read(addr,cmd,ENCODER_BUF_SIZE,buffer);
    union stringtofloat {
     uint8_t strbyte[4];
     float fval;
    } strtof;
    
    if(buffer[ENCODER_BUF_SIZE-1]=='\n'){
      for(int i=0;i<4;i++){
        strtof.strbyte[0] = buffer[(4*i) + 0];
        strtof.strbyte[1] = buffer[(4*i) + 1];
        strtof.strbyte[2] = buffer[(4*i) + 2];
        strtof.strbyte[3] = buffer[(4*i) + 3];
        switch(i){
          case 0: joint[0] = strtof.fval; break;
          case 1: joint[1] = strtof.fval; break;
          case 2: joint[2] = strtof.fval; break;
          case 3: joint[4] = strtof.fval; break;
        }
      }
    }
    return status;
  }

  
  void I2CPositionEncodersMgr::M866() {
    SERIAL_ECHOPAIR_F("SAD = J : ", position_joint_SAD[Joint1_AXIS]);
    SERIAL_ECHOPAIR_F(      "  A : ", position_joint_SAD[Joint2_AXIS]);
    SERIAL_ECHOPAIR_F(      "  B : ", position_joint_SAD[Joint3_AXIS]);
    SERIAL_ECHOLNPAIR_F(    "  D : ", position_joint_SAD[Joint5_AXIS]);
  }
/*
  void I2CPositionEncodersMgr::report_position(const int8_t idx, const bool units, const bool noOffset) {
    CHECK_IDX();

    if (units)
      SERIAL_ECHOLN(noOffset ? encoders[idx].mm_from_count(encoders[idx].get_raw_count()) : encoders[idx].get_position_mm());
    else {
      if (noOffset) {
        const int32_t raw_count = encoders[idx].get_raw_count();
        SERIAL_ECHO(axis_codes[encoders[idx].get_axis()]);
        SERIAL_CHAR(' ');

        for (uint8_t j = 31; j > 0; j--)
          SERIAL_ECHO((bool)(0x00000001 & (raw_count >> j)));

        SERIAL_ECHO((bool)(0x00000001 & raw_count));
        SERIAL_CHAR(' ');
        SERIAL_ECHOLN(raw_count);
      }
      else
        SERIAL_ECHOLN(encoders[idx].get_position());
    }
  }

  
  
  int8_t I2CPositionEncodersMgr::parse() {
    I2CPE_addr = 0;

    if (parser.seen('A')) {

      if (!parser.has_value()) {
        SERIAL_PROTOCOLLNPGM("?A seen, but no address specified! [30-200]");
        return I2CPE_PARSE_ERR;
      };

      I2CPE_addr = parser.value_byte();
      if (!WITHIN(I2CPE_addr, 30, 200)) { // reserve the first 30 and last 55
        SERIAL_PROTOCOLLNPGM("?Address out of range. [30-200]");
        return I2CPE_PARSE_ERR;
      }

      I2CPE_idx = idx_from_addr(I2CPE_addr);
      if (I2CPE_idx >= I2CPE_ENCODER_CNT) {
        SERIAL_PROTOCOLLNPGM("?No device with this address!");
        return I2CPE_PARSE_ERR;
      }
    }
    else if (parser.seenval('I')) {

      if (!parser.has_value()) {
        SERIAL_PROTOCOLLNPAIR("?I seen, but no index specified! [0-", I2CPE_ENCODER_CNT - 1);
        SERIAL_PROTOCOLLNPGM("]");
        return I2CPE_PARSE_ERR;
      };

      I2CPE_idx = parser.value_byte();
      if (I2CPE_idx >= I2CPE_ENCODER_CNT) {
        SERIAL_PROTOCOLLNPAIR("?Index out of range. [0-", I2CPE_ENCODER_CNT - 1);
        SERIAL_ECHOLNPGM("]");
        return I2CPE_PARSE_ERR;
      }

      I2CPE_addr = encoders[I2CPE_idx].get_address();
    }
    else
      I2CPE_idx = 0xFF;

    I2CPE_anyaxis = parser.seen_axis();

    return I2CPE_PARSE_OK;
  };

  
  // * M860:  Report the position(s) of position encoder module(s).
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
  // *   O        Include homed zero-offset in returned position.
  // *   U        Units in mm or raw step count.
  // *
  // *   If A or I not specified:
  // *    X       Report on X axis encoder, if present.
  // *    Y       Report on Y axis encoder, if present.
  // *    Z       Report on Z axis encoder, if present.
  // *    E       Report on E axis encoder, if present.
  // *
   
  void I2CPositionEncodersMgr::M860() {
    if (parse()) return;

    const bool hasU = parser.seen('U'), hasO = parser.seen('O');

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_position(idx, hasU, hasO);
        }
      }
    }
    else
      report_position(I2CPE_idx, hasU, hasO);
  }


  // * M861:  Report the status of position encoder modules.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
  // *
  // *   If A or I not specified:
  // *    X       Report on X axis encoder, if present.
  // *    Y       Report on Y axis encoder, if present.
  // *    Z       Report on Z axis encoder, if present.
  // *    E       Report on E axis encoder, if present.
  // *
   
  void I2CPositionEncodersMgr::M861() {
    if (parse()) return;

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_status(idx);
        }
      }
    }
    else
      report_status(I2CPE_idx);
  }

  
  // * M862:  Perform an axis continuity test for position encoder
  // *        modules.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
  // *
  // *   If A or I not specified:
  // *    X       Report on X axis encoder, if present.
  // *    Y       Report on Y axis encoder, if present.
  // *    Z       Report on Z axis encoder, if present.
  // *    E       Report on E axis encoder, if present.
  // *
  
  void I2CPositionEncodersMgr::M862() {
    if (parse()) return;

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) test_axis(idx);
        }
      }
    }
    else
      test_axis(I2CPE_idx);
  }

  
  // * M863:  Perform steps-per-mm calibration for
  // *        position encoder modules.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1]
  // *   P        Number of rePeats/iterations.
  // *
  // *   If A or I not specified:
  // *    X       Report on X axis encoder, if present.
  // *    Y       Report on Y axis encoder, if present.
  // *    Z       Report on Z axis encoder, if present.
  // *    E       Report on E axis encoder, if present.
  // *
   
  void I2CPositionEncodersMgr::M863() {
    if (parse()) return;

    const uint8_t iterations = constrain(parser.byteval('P', 1), 1, 10);

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) calibrate_steps_mm(idx, iterations);
        }
      }
    }
    else
      calibrate_steps_mm(I2CPE_idx, iterations);
  }

  
  // * M864:  Change position encoder module I2C address.
  // *
  // *   A<addr>  Module current/old I2C address.  If not present,
  // *            assumes default address (030).  [30, 200].
  // *   S<addr>  Module new I2C address. [30, 200].
  // *
  // *   If S is not specified:
  // *    X       Use I2CPE_PRESET_ADDR_X (030).
  // *    Y       Use I2CPE_PRESET_ADDR_Y (031).
  // *    Z       Use I2CPE_PRESET_ADDR_Z (032).
  // *    E       Use I2CPE_PRESET_ADDR_E (033).
   
  void I2CPositionEncodersMgr::M864() {
    uint8_t newAddress;

    if (parse()) return;

    if (!I2CPE_addr) I2CPE_addr = I2CPE_PRESET_ADDR_X;

    if (parser.seen('S')) {
      if (!parser.has_value()) {
        SERIAL_PROTOCOLLNPGM("?S seen, but no address specified! [30-200]");
        return;
      };

      newAddress = parser.value_byte();
      if (!WITHIN(newAddress, 30, 200)) {
        SERIAL_PROTOCOLLNPGM("?New address out of range. [30-200]");
        return;
      }
    }
    else if (!I2CPE_anyaxis) {
      SERIAL_PROTOCOLLNPGM("?You must specify S or [XYZE].");
      return;
    }
    else {
           if (parser.seen('X')) newAddress = I2CPE_PRESET_ADDR_X;
      else if (parser.seen('Y')) newAddress = I2CPE_PRESET_ADDR_Y;
      else if (parser.seen('Z')) newAddress = I2CPE_PRESET_ADDR_Z;
      else if (parser.seen('E')) newAddress = I2CPE_PRESET_ADDR_E;
      else return;
    }

    SERIAL_ECHOPAIR("Changing module at address ", I2CPE_addr);
    SERIAL_ECHOLNPAIR(" to address ", newAddress);

    change_module_address(I2CPE_addr, newAddress);
  }

  
  // * M865:  Check position encoder module firmware version.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
  // *
  // *   If A or I not specified:
  // *    X       Check X axis encoder, if present.
  // *    Y       Check Y axis encoder, if present.
  // *    Z       Check Z axis encoder, if present.
  // *    E       Check E axis encoder, if present.
 
  void I2CPositionEncodersMgr::M865() {
    if (parse()) return;

    if (!I2CPE_addr) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_module_firmware(encoders[idx].get_address());
        }
      }
    }
    else
      report_module_firmware(I2CPE_addr);
  }

  
  // * M866:  Report or reset position encoder module error
  // *        count.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
  // *   R        Reset error counter.
  // *
  // *   If A or I not specified:
  // *    X       Act on X axis encoder, if present.
  // *    Y       Act on Y axis encoder, if present.
  // *    Z       Act on Z axis encoder, if present.
  // *    E       Act on E axis encoder, if present.
   
  void I2CPositionEncodersMgr::M866() {
    if (parse()) return;

    const bool hasR = parser.seen('R');

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) {
            if (hasR)
              reset_error_count(idx, AxisEnum(i));
            else
              report_error_count(idx, AxisEnum(i));
          }
        }
      }
    }
    else if (hasR)
      reset_error_count(I2CPE_idx, encoders[I2CPE_idx].get_axis());
    else
      report_error_count(I2CPE_idx, encoders[I2CPE_idx].get_axis());
  }

  
  // * M867:  Enable/disable or toggle error correction for position encoder modules.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
  // *   S<1|0>   Enable/disable error correction. 1 enables, 0 disables.  If not
  // *            supplied, toggle.
  // *
  // *   If A or I not specified:
  // *    X       Act on X axis encoder, if present.
  // *    Y       Act on Y axis encoder, if present.
  // *    Z       Act on Z axis encoder, if present.
  // *    E       Act on E axis encoder, if present.
   
  void I2CPositionEncodersMgr::M867() {
    if (parse()) return;

    const int8_t onoff = parser.seenval('S') ? parser.value_int() : -1;

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) {
            const bool ena = onoff == -1 ? !encoders[I2CPE_idx].get_ec_enabled() : !!onoff;
            enable_ec(idx, ena, AxisEnum(i));
          }
        }
      }
    }
    else {
      const bool ena = onoff == -1 ? !encoders[I2CPE_idx].get_ec_enabled() : !!onoff;
      enable_ec(I2CPE_idx, ena, encoders[I2CPE_idx].get_axis());
    }
  }

  
  // * M868:  Report or set position encoder module error correction
  // *        threshold.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
  // *   T        New error correction threshold.
  // *
  // *   If A not specified:
  // *    X       Act on X axis encoder, if present.
  // *    Y       Act on Y axis encoder, if present.
  // *    Z       Act on Z axis encoder, if present.
  // *    E       Act on E axis encoder, if present.
   
  void I2CPositionEncodersMgr::M868() {
    if (parse()) return;

    const float newThreshold = parser.seenval('T') ? parser.value_float() : -9999;

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) {
            if (newThreshold != -9999)
              set_ec_threshold(idx, newThreshold, encoders[idx].get_axis());
            else
              get_ec_threshold(idx, encoders[idx].get_axis());
          }
        }
      }
    }
    else if (newThreshold != -9999)
      set_ec_threshold(I2CPE_idx, newThreshold, encoders[I2CPE_idx].get_axis());
    else
      get_ec_threshold(I2CPE_idx, encoders[I2CPE_idx].get_axis());
  }
 
  
  // * M869:  Report position encoder module error.
  // *
  // *   A<addr>  Module I2C address.  [30, 200].
  // *   I<index> Module index.  [0, I2CPE_ENCODER_CNT - 1].
  // *
  // *   If A not specified:
  // *    X       Act on X axis encoder, if present.
  // *    Y       Act on Y axis encoder, if present.
  // *    Z       Act on Z axis encoder, if present.
  // *    E       Act on E axis encoder, if present.
  
  void I2CPositionEncodersMgr::M869() {
    if (parse()) return;

    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_error(idx);
        }
      }
    }
    else
      report_error(I2CPE_idx);
  }
//*/
#endif // I2C_POSITION_ENCODERS
