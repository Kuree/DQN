#include <stdio.h>
#include <wiringPi.h>

int main(void){
  wiringPiSetup();
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  for (;;){

    printf("ON\n");
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    printf("4=%d 5=%d\n",digitalRead(4),digitalRead(5));
    delay(250);
    printf("OFF\n");
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    printf("4=%d 5=%d\n",digitalRead(4),digitalRead(5));
    delay(350);
  }
}
