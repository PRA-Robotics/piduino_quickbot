/* 
 *  SKetch accepts numerical versions of Pysimiam commands
 *  08july2019 version
 */
#include <RedBot.h>

RedBotMotors motors;
int sensorPin=A6;  // initialize a analog sensor object on A6
RedBotEncoder encoder = RedBotEncoder(A1, A4);

int dot_time_unit = 50;          // delay time for dot in msec

void setup()
{
  // initialize serial port
  Serial.begin(9600);    // serial / USB port
  while (!Serial) { // wait for seriol port to be ready 
    }
  
  //initialize motors
  motors.stop();
  encoder.clearEnc(BOTH);

  // setup LED for morse codes
  pinMode(13, OUTPUT);
}

void display_morsecode(String code)
{
  int code_len = code.length();
  int dot_time = dot_time_unit;
  int dash_time = dot_time_unit *3;
  for (int i=0; i<code_len; i++) {
    if (code.charAt(i) == '.') {
      digitalWrite(13, HIGH);             // Turn on the LED for dot
      delay(dot_time);               // Usec for a 1* dot time period
    } else { // then it's a dash
      digitalWrite(13, HIGH);               // Turn on the LED for a dash
      delay(dash_time);               // a dash is 3 * dot time period
    }
    digitalWrite(13, LOW);      // turn off for one dot time unit after a Dot|Dash
    delay(dot_time_unit);
    } // end for code
  digitalWrite(13, LOW);
  delay(dash_time);                 // turn off for 3 (2+1 @end of char) dot times units after character
} // end display_morsecode
    
void string2morse(String error_string)
{ 
  int err_str_len = error_string.length();
  int maxmorsechar = 72;    // number of characters in morse we support
  
  String morsetable[maxmorsechar]={"Aa", ".-", "Bb", "-...", "Cc", "-.-.", "Dd", "-..", "Eo", ".", "Ff", 
  "..-.", "Gg", "--..", "Hh", "....", "Ii", "..", "Jj", ".---", "Kk", "-.-", "Ll", ".-..",
  "Mm", "--", "Nn", "-.","Oo", "---", "Pp", ".--.", "Qq", "--.-", "Rr", ".-.",
  "Ss", "...", "Tt", "-", "Uu", "..-", "Vv", "...-", "Ww", ".--", "Xx", "-..-", "Yy", "-.--", "Zz", "--..",
  "11", ".----", "22", "..---", "33", "...--", "44", "....-", "55", ".....", 
  "66", "-....", "77", "--...", "88", "---..", "99", "----.", "00", "-----"}; 
       
  for (int j=0; j< err_str_len; j++) {
    for (int i=0; i<maxmorsechar; i=i+2) {
      if ( (error_string.charAt(j) == morsetable[i].charAt(0)) or (error_string.charAt(j) == morsetable[i].charAt(1)) ) {
        display_morsecode(morsetable[i+1]); 
      } // end if
    } // end moresetable scan
  } // end chars2display loop
  digitalWrite(13, LOW);              // Turn off the LED  
  delay(7*dot_time_unit);             // delay 7 dots between words 
} // end string2morse

void gen_response5(int resp, int i1, int i2, int i3, int i4, int i5)
{ 
  Serial.print(resp, DEC);
  Serial.print(" ");
  Serial.print(5, DEC);
  Serial.print(" ");
  Serial.print(i1, DEC);
  Serial.print(" ");
  Serial.print(i2, DEC);
  Serial.print(" ");
  Serial.print(i3, DEC);
  Serial.print(" ");
  Serial.print(i4, DEC);
  Serial.print(" ");
  Serial.println(i5, DEC);
}

void gen_response2(int resp, int i1, int i2)
{ 
  Serial.print(resp, DEC);
  Serial.print(" ");
  Serial.print(2, DEC);
  Serial.print(" ");
  Serial.print(i1, DEC);
  Serial.print(" ");
  Serial.println(i2, DEC);
}

void gen_response2_float(int resp, float f1, float f2)
{ 
  Serial.print(resp, DEC);
  Serial.print(" ");
  Serial.print(2, DEC);
  Serial.print(" ");
  Serial.print(f1, 4); // to four decimal places
  Serial.print(" ");
  Serial.println(f2, 4); // to four decimal places
}

void gen_response0(int resp)
{ 
  Serial.print(resp, DEC);
  Serial.print(" ");
  Serial.println(0, DEC);
}
void loop()
{
  int command_code=0; 
  String CMD_response = "";
  int len_input=0;
  int ir0=1024, ir1=1024, ir2=0, ir3=1024, ir4=1024; // infared sensor voltage return, 133 is default for no obstacles being sensed
  
  String morse2display="012345678901234567890";
  
  const int RESET=10, PWM_set=20, PWM_enq=30, ENVAL_enq=40, ENVEL_enq=50, IRVAL_enq=60, CHECK=70, ENC_test=255;
        // Define command codes
  const int CMD_ok=0, CMD_unk=100, INT_zero=110, INT_one=120, INT_out_of_range = 130;
  int PWM_left = 0, PWM_right=0; 
  long PWM_left_inp=0, PWM_right_inp=0, ENC_left=0, ENC_right=0;
  const float WHEEL_RADIUS = 6.5/2; // CM as measured by Ray
  const float TICKS_PER_REVOLUTION = 189.0; // ranges from 178 to 200 but best luck with 189?
  float VEL_left = 0.0, VEL_right= 0.0, ENC_left_delta = 0.0, ENC_right_delta=0.0;
  
  digitalWrite(13, LOW);
  
  // check if command available from USB port
  if (Serial) {
    len_input=Serial.available();
    if (len_input > 2) {
      command_code = Serial.parseInt();
      display_morsecode(String(command_code));
     
      // perform function given across serial port
      switch (command_code) {
        case RESET: {
          // reset all robot sensors, motors, etc.
          motors.rightMotor(0);
          motors.leftMotor(0);
          PWM_left=0;
          PWM_right-0;
          gen_response0(CMD_ok);
          Serial.write("RESET Rcvd \n");
          break;
        }
        case PWM_set: {
          // set motor power to L, R
          PWM_left_inp= Serial.parseInt();
          PWM_right_inp= Serial.parseInt();
          // display_morsecode(String(PWM_left_inp + "SOS" + PWM_right_inp));
          if (PWM_left_inp == 0 && PWM_right_inp == 0) {
            gen_response0(INT_zero);
            break;
          }
          if (PWM_right_inp == 0) { 
            gen_response0(INT_one);
            break;
          }
          if (PWM_left_inp == 256) { // we use 256 to indicate a power level of 0
            PWM_left_inp = 0; 
          }
          if (PWM_right_inp == 256)  { // we use 256 to indicate a power level of 0
            PWM_right_inp = 0; 
          }
          if (PWM_left_inp > -255 && PWM_left_inp < 255 && PWM_right_inp > -255 && PWM_right_inp < 255) {
            // PWM settings have to be in range -255..+255
            PWM_left = PWM_left_inp; // save the power values as there's no way of reading them,
            PWM_right = PWM_right_inp;
            motors.leftMotor(PWM_left_inp);
            motors.rightMotor(PWM_right_inp);
            
            gen_response2(CMD_ok, PWM_left, PWM_right);
            Serial.write("Set PWM rcvd \n");
            break;
          } else {
            // some power value is out of range
            gen_response2(INT_out_of_range, PWM_left, PWM_right);
            break;
          }
        } // end power motor set command functionality
        case PWM_enq: {
          // send motor power values L,r saved from last power set cmd;
          
          gen_response2(CMD_ok, PWM_left, PWM_right);
          Serial.write("PWM query rcvd \n");
          break;
        }
        case ENVAL_enq: {
          // send motor encoder values L, R
          ENC_left= encoder.getTicks(LEFT);
          ENC_right= encoder.getTicks(RIGHT);
          
          gen_response2(CMD_ok, ENC_left, ENC_right);
          Serial.write("Encoder query rcvd \n");
          break;
        }
        case ENVEL_enq: {
          // send motor velocity values L, R
          ENC_left= encoder.getTicks(LEFT);
          ENC_right= encoder.getTicks(RIGHT);
          
          delay(1000); // wait one second to see how many ticks have transpired

          ENC_left_delta = (float) (encoder.getTicks(LEFT) - ENC_left);
          ENC_right_delta = (float) (encoder.getTicks(RIGHT) - ENC_right);
          VEL_left = (ENC_left_delta/TICKS_PER_REVOLUTION)*2.0*PI*WHEEL_RADIUS;
          VEL_right = (ENC_right_delta/TICKS_PER_REVOLUTION)*2.0*PI*WHEEL_RADIUS;
          
          gen_response2_float(CMD_ok, VEL_left, VEL_right);
          Serial.write("Wheel velocity query rcvd \n");
          break;
        }
        case IRVAL_enq: {
          // send IR sensor values IR0, IR1, IR2, IR3, IR4, IR5
          // we will use SENTINAL value for all IR sensors that don't exist
          // IR2 iwill be returned from an analog read 
          
          gen_response5(CMD_ok, ir0, ir1, ir2, ir3, ir4);
          Serial.write("IR sensor query rcvd \n");
          break;
        }
        case CHECK: {
          // Clear encoders and send back command rcvd & processed good status
          encoder.clearEnc(BOTH);
          
          gen_response0(CMD_ok);
          Serial.write("Check command rcvd \n");
          break;
        }
        case ENC_test: { // a tester to run to encoder for L and R wheel
          // stop motors
          motors.leftMotor(0);
          motors.rightMotor(0);

          // figure out what the encoder value we need to reach 
          ENC_left_delta = Serial.parseInt(); // get encoder values for both wheels.
          ENC_right_delta = Serial.parseInt();
          ENC_left_delta = ENC_left_delta + encoder.getTicks(LEFT);
          ENC_right_delta = ENC_right_delta + encoder.getTicks(RIGHT);

          // test left motor encoder 
          ENC_left = encoder.getTicks(LEFT);
          while (ENC_left < ENC_left_delta) {
            motors.leftMotor(-32);
            delay(100);
            ENC_left=encoder.getTicks(LEFT);
          }
          motors.leftMotor(0);
          delay(5000);
          
          // test right motor encoder 
          ENC_right=encoder.getTicks(RIGHT);
          while (ENC_right < ENC_right_delta) {
            motors.rightMotor(32);
            delay(50);
            ENC_right=encoder.getTicks(RIGHT);
          }
          motors.rightMotor(0);
          
          gen_response0(CMD_ok);
          break;
        }
        default: {
          gen_response0(CMD_unk);
          break;
        }
      } // end switch
    } // end end if len > 1   
  } // if CSeriol)

}
