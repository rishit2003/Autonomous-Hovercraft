
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <util/twi.h>


#define SCL_CLOCK 100000L
#define F_CPU 16000000L

#define MPU6050_ADDRESS 0x68   // Device address when ADO = 0
#define WHO_AM_I_MPU6050 0x75  // Should return 0x68

#define A_X_H 0x3B
#define A_X_L 0x3C
#define A_Y_H 0x3D
#define A_Y_L 0x3E
#define A_Z_H 0x3F
#define A_Z_L 0x40
#define ACC_CONFIG 0x1C  //access full scale range bits4,3


#define GY_X_H 0x43
#define GY_X_L 0x44
#define GY_Y_H 0x45
#define GY_Y_L 0x46
#define GY_Z_H 0x47
#define GY_Z_L 0x48
#define GYR_CONFIG 0x1B  //access full scale range bits4,3


//accelerometer sensitivity - full-scale range -
typedef enum {  //AFS_SEL bit
  fsr_2g = 0,   //scale factor 16384 LSB/g
  fsr_4g = 1,   //scale factor 8192 LSB/g
  fsr_8g = 2,   //scale factor 4096 LSB/g
  fsr_16g = 3   //scale factor 2048 LSB/g
} Accel_range;

//gyroscope sensitivity - full-scale range
typedef enum {      //AFS_SEL bit
  fsr_250dps = 0,   //scale factor  131 LSB/dps
  fsr_500dps = 1,   //scale factor 65.5 LSB/dps
  fsr_1000dps = 2,  //scale factor 32.8 LSB/dps
  fsr_2000dps = 3   //scale factor 16.4 LSB/dps
} Gyro_range;


void twi_init(void) {
  // TWI init
  TWSR = 0;                                           // no prescaler
  TWBR = (uint8_t)(((F_CPU / SCL_CLOCK) - 16) >> 1);  //setting SCL; must be >10 for stable operation
}

volatile struct {
  uint8_t TX_new_data : 1;
  uint8_t TX_finished : 1;
  uint8_t TX_buffer1_empty : 1;
  uint8_t TX_buffer2_empty : 1;
  uint8_t RX_flag : 3;
  uint8_t TWI_ACK : 1;
} flags;

volatile uint8_t TWI_status;  //
volatile uint8_t TWI_byte;
volatile unsigned long count;

int16_t a_x ;  //acc
int16_t a_y ;  //acc
int16_t a_z ;  //acc



int16_t g_x ;  //gyro
int16_t g_y ;  //gyro
int16_t g_z;      //gyro

float roll = 0;   //acc
float pitch = 0;  //acc
float yaw = 0;    //acc



unsigned long startTime;    // To store the start time
unsigned long elapsedTime;  // To store the elapsed time
unsigned long currentTime;

//---------------------------------------------------------------------------------------


//=============================== TWI functions ======================

uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write) {

  TWCR = ((1 << TWINT) | (1 << TWSTA) | (1 << TWEN));  //send START condition
  while (!(TWCR & (1 << TWINT)))
    ;                                                                            //wait until transmission completed
  if (((TWSR & 0xF8) != TW_START) && ((TWSR & 0xF8) != TW_REP_START)) return 1;  //something went wrong

  twi_addr = ((twi_addr << 1) | (read_write & 1));  //setting r/w bit
  TWDR = twi_addr;                                  //send device address
  TWCR = ((1 << TWINT) | (1 << TWEN));              //reset the flag

  while (!(TWCR & (1 << TWINT)))
    ;  // wail until transmission completed and ACK/NACK has been received
  if (((TWSR & 0xF8) != TW_MT_SLA_ACK) && ((TWSR & 0xF8) != TW_MR_SLA_ACK)) return 2;
  return 0;
}

void inline TWI_stop(void) {
  TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));  // send stop condition
  while (TWCR & (1 << TWSTO))
    ;  // wait until stop condition is executed and bus released
}

uint8_t TWI_write(uint8_t tx_data)  //write byte to the started device
{
  TWDR = tx_data;
  TWCR = ((1 << TWINT) | (1 << TWEN));
  while (!(TWCR & (1 << TWINT)))
    ;                                             // wait until transmission completed
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 1;  //check value of TWI Status Register. Mask prescaler bits. Write failed
  return 0;
}

uint8_t TWI_ack_read(void)  // continuous read
{
  TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWEA));  // Start read cycle
  while (!(TWCR & (1 << TWINT)))
    ;
  flags.TWI_ACK = 1;
  if ((TWSR & 0xF8) != TW_MR_DATA_ACK) flags.TWI_ACK = 0;  //check value of TWI Status Register. Mask prescaler bits. Read failed
  return TWDR;
}


uint8_t TWI_nack_read(void)  //read and stop condition
{
  TWCR = ((1 << TWINT) | (1 << TWEN));
  while (!(TWCR & (1 << TWINT)))
    ;
  flags.TWI_ACK = 1;
  if ((TWSR & 0xF8) != TW_MR_DATA_NACK) return flags.TWI_ACK = 0;  //check value of TWI Status Register. Mask prescaler bits. Read failed
  return TWDR;
}


uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr) {

  //uint8_t data;  // added to original code

  TWI_status = TWI_start(TWI_addr, TW_WRITE);
  if (TWI_status) return 1;

  TWI_status = TWI_write(reg_addr);  //  register #
  if (TWI_status) return 2;

  TWI_status = TWI_start(TWI_addr, TW_READ);
  if (TWI_status) return 3;

  TWI_byte = TWI_nack_read();
  // data = TWI_byte;
  TWI_stop();
  if (!flags.TWI_ACK) return 4;
  return 0;
}



uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, uint16_t* data) {
  uint8_t temp_data[sizeof(uint16_t)];  // Temporary storage for 8-bit data
  uint8_t i;

  // Start TWI communication for writing
  TWI_status = TWI_start(TWI_addr, TW_WRITE);
  if (TWI_status) return 1;

  // Write the register address
  TWI_status = TWI_write(reg_addr);
  if (TWI_status) return 2;

  // Start TWI communication for reading
  TWI_status = TWI_start(TWI_addr, TW_READ);
  if (TWI_status) return 3;

  // Read data bytes (except the last one)
  for (i = 0; i < bytes - 1; i++) {
    temp_data[i] = TWI_ack_read();
    if (!flags.TWI_ACK) return 5;
  }

  // Read the last byte
  temp_data[i] = TWI_nack_read();

  // Combine the two bytes into a 16-bit value
  *data = (uint16_t)((temp_data[0] << 8) | temp_data[1]);

  // Stop TWI communication
  TWI_stop();
  if (!flags.TWI_ACK) return 4;

  return 0;
}


uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value) {

  TWI_status = TWI_start(TWI_addr, TW_WRITE);
  if (TWI_status) return 1;

  TWI_status = TWI_write(reg_addr);  //  register #
  if (TWI_status) return 2;

  TWI_status = TWI_write(value);  // write the value
  if (TWI_status) return 3;

  TWI_stop();
  if (!flags.TWI_ACK) return 4;

  return 0;
}

void D3_brightness_accelerometer_X(int16_t ax) {
  if (abs(ax / 16384.0f) < 0.08) {
    OCR2A = 0;  // D3 off
  } else if (abs(ax / 16384.0f) > 1.08) {  //D3 on
    OCR2A = 255;
  } else {
    
    OCR2A = (255 - abs(ax / 16384.0f) * 10);   //PWM on the pin with OC2A 
  }
}

void Onboard_led_IMU_yaw(float yaw) {
  if (!(abs(yaw) < 85)) {
    PORTB |= (1 << 5);  // Writing HIGH to PB5  
  } else {
     PORTB &= ~(1 << 5);  // Writing LOW to PB5
  }
}

//Timer1 compare match A interrupt handler (runs every 1 ms)
ISR(TIMER1_COMPA_vect) {
}

int main(void) {
  Serial.begin(9600);


  // set LEDs 
   DDRB |= (1 << PB3);    //  red led
   DDRB |= (1 << PB5);    //  onboard  yellow led



  // Set SDA and SCL pins
  DDRC &= ~((1 << PC4) | (1 << PC5));

  // Enable internal pull-up resistors for SDA and SCL
  PORTC |= ((1 << PC4) | (1 << PC5));

  //          ----------Configure Timer1 for 1 ms interrupts use to calculate elapsed time
  TCCR1A = 0;                           // Normal mode
  TCCR1B = (1 << CS10) | (1 << WGM12);  // Prescaler = 1, CTC mode
  OCR1A = 15999;                        // Compare value for 1 ms interrupt
  TIMSK1 = (1 << OCIE1A);               // Enable Timer1 compare match A interrupt

  //-------------------------------------------------------------------------------- Enable global interrupts
  sei();

  //--- for D3 on PB3 to generate PWM
  OCR2A = 100;
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << WGM21) | (1 << WGM20);   // fast pwm, non inverting mode
  TCCR2B = (1 << CS22) | (1 << CS20);   //128 prescaler
  //----


  // TWI initialisation
  twi_init();  // set the clock speed

 



  //TWINT bit is cleared by writing a one
  //TWSTA to 1 tells the TWI we want to send a start
  //TWEN enables the TWI
  //TWEA enables send “acknowledge” after a data packet is received
  // pattern -->Set TWCR, wait for TWINT, check TWSR.


  //power management
  Write_Reg(0x68, 0x6B, 0);  //write to pwr_management register to wake up


  // Set accelerometer configuration to +-2g
  uint8_t Accel_range = fsr_2g;

  Read_Reg(MPU6050_ADDRESS, ACC_CONFIG);
  uint8_t data = TWI_byte;

  Write_Reg(MPU6050_ADDRESS, ACC_CONFIG, data & ~0x18);             // clear bits 4,3
  Write_Reg(MPU6050_ADDRESS, ACC_CONFIG, data | Accel_range << 3);  // Set full scale range for the accelerometer

  // Set gyroscope configuration to +-250dps

  Write_Reg(MPU6050_ADDRESS, 0x1A, 0x02);  // set low pass filter
  uint8_t Gyro_range = fsr_250dps;

  Read_Reg(MPU6050_ADDRESS, GYR_CONFIG);
  uint8_t data_gyr = TWI_byte;
  Write_Reg(MPU6050_ADDRESS, GYR_CONFIG, data_gyr & ~0x18);            // clear bits 4,3
  Write_Reg(MPU6050_ADDRESS, GYR_CONFIG, data_gyr | Gyro_range << 3);  // Set full scale range for the accelerometer

  _delay_ms(10);

  // calculate offset for acc in X Y Z
  int32_t a_x_off_rev = 0;
  int32_t a_y_off_rev = 0;
  int32_t a_z_off_rev = 0;

  // calculate offset for gyro in X Y Z
  int32_t g_x_off_rev = 0;
  int32_t g_y_off_rev = 0;
  int32_t g_z_off_rev = 0;


  for (int i = 0; i < 2000; i++) {
    Read_Reg_N(0x68, A_X_H, 2, &a_x);  //AcX
    Read_Reg_N(0x68, A_Y_H, 2, &a_y);  //AcY
    Read_Reg_N(0x68, A_Z_H, 2, &a_z);  //AcZ


    Read_Reg_N(0x68, GY_X_H, 2, &g_x);  //AcX
    Read_Reg_N(0x68, GY_Y_H, 2, &g_y);  //AcY
    Read_Reg_N(0x68, GY_Z_H, 2, &g_z);  //AcZ


    a_x_off_rev += (int32_t)a_x;
    a_y_off_rev += (int32_t)a_y;
    a_z_off_rev += (int32_t)a_z;

    g_x_off_rev += (int32_t)g_x;
    g_y_off_rev += (int32_t)g_y;
    g_z_off_rev += (int32_t)g_z;


    _delay_ms(2);
  }




  Serial.println(g_x_off_rev);
  g_x_off_rev /= 2000;
  Serial.println(g_x_off_rev);
  g_y_off_rev /= 2000;
  g_z_off_rev /= 2000;


  //--------------  end for offset calculation


  float accelAngle_X;
  float accelAngle_Y;
  float accelAngle_Z;



  while (1) {  // to read data continuously



    Read_Reg_N(0x68, A_X_H, 2, &a_x);
    Read_Reg_N(0x68, A_Y_H, 2, &a_y);  //AcY
    Read_Reg_N(0x68, A_Z_H, 2, &a_z);  //AcZ

    D3_brightness_accelerometer_X(a_x);

    accelAngle_X = (atan(a_y / 16384.0 / sqrt(pow(a_x / 16384.0, 2) + pow(a_z / 16384.0, 2))) * 180 / PI);         
    accelAngle_Y = (atan(-1 * (a_x / 16384.0) / sqrt(pow(a_y / 16384.0, 2) + pow(a_z / 16384.0, 2))) * 180 / PI);  
                                                                                                                   // accelAngle_Z = (atan(sqrt(pow(a_x/ 16384.0,2) + pow(a_y/ 16384.0,2))/(a_z/ 16384.0))* 180 / PI);

    Serial.print("-----------------------------------------roll and pitch using accelerometer");

    Serial.print(accelAngle_X);
    Serial.print("/");
    Serial.print(accelAngle_Y);
    Serial.println("/");
   



    startTime = currentTime;
    currentTime = TCNT1;
    if (currentTime >= startTime) {
      elapsedTime = currentTime - startTime;
    } else {
      // Handle timer overflow (wraparound)
      elapsedTime = (65536 - startTime) + currentTime;
    }

    // Convert elapsed time to seconds (assuming 16 MHz clock and no prescaling)
    float elapsedSeconds = static_cast<float>(elapsedTime) / 16000000.0;

    // Serial.print("Elapsed time (s): ");
    // Serial.println(elapsedSeconds,6);
    // Serial.println(startTime);
    // Serial.println(currentTime);
    //---------------------------------------------------------------------------------




    // //Start and read gyroscope X,Y,Z

    Read_Reg_N(0x68, GY_X_H, 2, &g_x);  //AcX
    Read_Reg_N(0x68, GY_Y_H, 2, &g_y);  //AcY
    Read_Reg_N(0x68, GY_Z_H, 2, &g_z);  //AcZ



    Serial.print("-----------------------------------------  a_x a_y a_z: ");
    Serial.print(a_x / 16384.0f);
    Serial.print(" / ");
    Serial.print(a_y / 16384.0f);
    Serial.print(" / ");
    Serial.println(a_z / 16384.0f);


    // Serial.print("------------------------------------------   AcX AcY AcZ with  offset = ");
    // Serial.print((a_x + a_x_off_rev) / 16384.0f);
    // Serial.print(" / ");
    // Serial.print((a_y + a_y_off_rev) / 16384.0f);
    // Serial.print(" / ");
    // Serial.print((a_z + a_z_off_rev) / 16384.0f);
    // Serial.println(" / ");


    //  Serial.print(" offset g_acc g_acc g_acc is: ");
    //   Serial.print(g_x_off_rev/ 131.0f,4);
    //     Serial.print(" / ");
    //     Serial.print(g_y_off_rev/ 131.0f,4);
    //     Serial.print(" / ");
    //     Serial.print(g_z_off_rev/ 131.0f,4);
    //    Serial.print(" / ");
    //     Serial.println(" ");




    roll += (g_x / 131.0f) * elapsedSeconds;
    pitch += (g_y / 131.0f) * elapsedSeconds;
    yaw += (g_z / 131.0f) * elapsedSeconds;
    Serial.print(" -------------------------------------------  roll pitch and yaw are: ");
    Serial.print(roll);
    Serial.print(" / ");
    Serial.print(pitch);
    Serial.print(" / ");
    Serial.println(yaw);
    Serial.println("  ");

    Onboard_led_IMU_yaw(yaw) ;


    _delay_ms(1000);
  }


  return 0;
}

