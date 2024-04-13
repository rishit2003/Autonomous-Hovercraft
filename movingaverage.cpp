
#include "movingaverage.h"

/*------------------- moving average filters variables to be set in main




int INDEX = 0;
volatile float VALUE = 0;
volatile float SUM = 0;
volatile float READINGS[WINDOW_SIZE];
volatile float AVERAGE = 0;
volatile float DISTANCE = 0;
*/

//uint8_t window_s;
volatile float value ;
volatile float sum ;
volatile float readings[20];
volatile float average ;
volatile float dist ;
volatile uint8_t index=0 ; 


float moving_average(float dist, uint8_t window_size){

   sum = sum - readings[index];       // remove the oldest entry from the sum
   value = dist;        // read the next sensor value
   readings[index] = value;           // add the newest reading to the window
   sum = sum + value;                 // add the newest reading to the sum
   index = (index+1) % window_size;   // increment the index, and wrap to 0 if it exceeds the window size
   average = sum / window_size;      // Divide the sum of the window by the window size for the result
  
  
  
  return average;
}