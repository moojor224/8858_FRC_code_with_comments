
// add these variable instantiations under alteredX and alteredY definitions at top of Robot.java
double last_sampled_alteredX = 0;
double last_sampled_alteredY = 0;
double last_sample_time = 0;
double sample_rate = 50;
double max_changeX = 1;
double max_changeY = 0.025;
boolean changeX_too_fast_flag = false;
boolean changeY_too_fast_flag = false;


      alteredX = modif2 * Math.pow(m_stick.getRightX(), 3) - ((modif2 - 1) * m_stick.getRightX());
      alteredY = modif1 * Math.pow(m_stick.getLeftY(), 3) - ((modif1 - 1) * m_stick.getLeftY());
      ///////////////////////////// END MOBILITY OPTION 1 /////////////////////////////

      ///////////////////////////// BEGIN RAMP TEST CODE /////////////////////////////

      // if the ΔX flag is set, override the alteredX until input is within the "max_changeX" of the last sampled value
      if(changeX_too_fast_flag){
        if((alteredX < (last_sampled_alteredX - max_changeX)) || (alteredX > (last_sampled_alteredX + max_changeX))){
          alteredX = last_sampled_alteredX;
        }else{
          System.out.println("INFO : ");
          changeX_too_fast_flag = false;
        }
      }

      // if the ΔY flag is set, override the alteredY until input is within the "max_changeY" of the last sampled value
      if(changeY_too_fast_flag){
        if((alteredY < (last_sampled_alteredY - max_changeY)) || (alteredY > (last_sampled_alteredY + max_changeY))){
          alteredY = last_sampled_alteredY;
        }else{
          System.out.println("INFO : ");
          changeY_too_fast_flag = false;
        }
      }

      /*  check the rate that alteredX and alteredY are     *
       *  changing at a "sample_rate" # of ms.              *
       *  "max_change" variables are the most the altered   *
       *  values can change in a "sample_rate" time period. */
      if(System.currentTimeMillis() > (last_sample_time + sample_rate)){
        last_sample_time = System.currentTimeMillis();

        // alteredX check
        if(alteredX < (last_sampled_alteredX - max_changeX)){
          alteredX = last_sampled_alteredX - max_changeX;
          last_sampled_alteredX = last_sampled_alteredX - max_changeX;
          System.out.println("INFO : ");
          changeX_too_fast_flag = true;
        }else if(alteredX > (last_sampled_alteredX + max_changeX)){
          alteredX = last_sampled_alteredX + max_changeX;
          last_sampled_alteredX = last_sampled_alteredX + max_changeX;
          System.out.println("INFO : ");
          changeX_too_fast_flag = true;
        } else {
          last_sampled_alteredX = alteredX;
          if(changeX_too_fast_flag){
            System.out.println("INFO : ");
            changeX_too_fast_flag = false;
          }
        }

        // alteredY check
        if(alteredY < (last_sampled_alteredY - max_changeY)){
          alteredY = last_sampled_alteredY - max_changeY;
          last_sampled_alteredY = last_sampled_alteredY - max_changeY;
          System.out.println("INFO : ");
          changeY_too_fast_flag = true;
        }else if(alteredY > (last_sampled_alteredY + max_changeY)){
          alteredY = last_sampled_alteredY + max_changeY;
          last_sampled_alteredY = last_sampled_alteredY + max_changeY;
          System.out.println("INFO : ");
          changeY_too_fast_flag = true;
        } else {
          last_sampled_alteredY = alteredY;
          if(changeY_too_fast_flag){
            System.out.println("INFO : ");
            changeY_too_fast_flag = false;
          }
        }
      }
      ////////////////////////////// END RAMP TEST CODE //////////////////////////////

      m_differentialDrive.arcadeDrive(-alteredY * MOTOR_SPEED_LIMIT, -alteredX * MOTOR_SPEED_LIMIT);