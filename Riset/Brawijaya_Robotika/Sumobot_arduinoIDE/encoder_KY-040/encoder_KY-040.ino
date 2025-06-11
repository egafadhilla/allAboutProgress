const int CLKPin_rotary = GPIO_NUM_14;
const int DTNPin_rotary =  GPIO_NUM_27;
int CLKState_rotary;
int lastCLKState_rotary = LOW;


int baca_rotary() {
CLKState_rotary = digitalRead(CLKPin_rotary);
int nilai = 0;
 
if (CLKState_rotary != lastCLKState_rotary) {
if (digitalRead(DTNPin_rotary) != CLKState_rotary) {
nilai = 1;
} else {
nilai = -1;
}
}
lastCLKState_rotary = CLKState_rotary;
return nilai;
}


void setup()
{
Serial.begin(9600);
pinMode(CLKPin_rotary, INPUT);
pinMode(DTNPin_rotary, INPUT);

}
void loop(){
int nilai = baca_rotary();
if (nilai != 0) {
Serial.print("Perubahan nilai: ");
Serial.println(nilai);
}


}
