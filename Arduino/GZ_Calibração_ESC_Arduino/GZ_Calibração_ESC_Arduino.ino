/////CALIBRAÇÃO DO ESC BIDIRECIONAL PARA O DRONE ABELHINHA. YOUTUBE: GRAVIDADE ZERO////
//-------------------------------------------------------------------------------------
//									Termos de uso									  |
//ESTE SOFTWARE FOI DISTRIBUÍDO EM SEU ESTADO ATUAL, SEM NENHUMA GARANTIA, DE FORMA	  |
//IMPLÍCITA OU EXPLÍCITA. DESIGNADO PARA USO PESSOAL.								  |
//OS AUTORES NÃO SÃO RESPONSÁVEIS POR QUALQUER DANO CAUSADO PELO USO DESTE SOFTWARE.  |
//-------------------------------------------------------------------------------------
//								NOTA DE SEGURANÇA									  |																
//SEMPRE REMOVA AS HÉLICES ANTES DE REALIZAR QUALQUER TESTE,						  |
//A MENOS QUE VOCÊ SAIBA O QUE ESTÁ FAZENDO.										  |
//-------------------------------------------------------------------------------------
//								NOTA DE SEGURANÇA 2									  |
//ESTE SOFTWARE APENAS FUNCIONA PARA ESCs BIDIRECIONAIS, OU SEJA,					  |
//ESC's EM QUE O PONTO MORTO DO MOTOR É 1500us DE LARGURA DE PULSO, CUIDADO.		  |
//O USO DE UM ESC UNIDIRECIONAL PODE RESULTAR EM ACIDENTES OU ATÉ PERDA TOTAL DOS	  |
//COMPONENTES, POIS O MOTOR NÃO RECEBERÁ O COMANDO PARA PARAR, OU O ESC NÃO IRÁ ARMAR.|
//-------------------------------------------------------------------------------------
//ESTE PROGRAMA FOI BASEADO NO TRABALHO PRÉVIO DE JOOP BROOKING, SENDO TRADUZIDO E	  |
//ADAPTADO PARA O PROJETO COM ESC BIDIRECIONAL LIMITADO À UMA DIREÇÃO				  |
//FONTE: http://www.brokking.net/ymfc-al_main.html									  |
//AGRADECIMENTOS AO JOOP BROOKING POR CONTRIBUIR COM A DISSEMINAÇÃO DE CONHECIMENTOS  |
//SOBRE O UNIVERSO DOS VANTS. CANAL: https://www.youtube.com/user/MacPuffdog/featured |
//-------------------------------------------------------------------------------------

			//=================================================================
			//COMANDOS SERIAIS PARA O TESTE OU CALIBRAÇÃO					  |
			//r = Mostrar os valores adquiridos pelo receptor				  |
			//a = Mostrar os valores convertidos da IMU			  			  |
			//1 = Checar a rotação e vibração do motor 1  (frontal direito)   |
			//2 = Checar a rotação e vibração do motor 2  (traseiro direito)  |
			//3 = Checar a rotação e vibração do motor 3  (traseiro esquerdo) |
			//4 = Checar a rotação e vibração do motor 4  (frontal esquerdo)  | 
			//5 = Checar vibração de todos os motores simultaneamente		  |
			//=================================================================

#include <Wire.h>        //Biblioteca para comunicação i²C com a IMU
#include <EEPROM.h>      //Comunicação com a memória EEPROM da ESP32

//Varíaveis globais
int debug = 0;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36], start, data;
boolean new_function_request,first_angle;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int receiver_input[5];
int loop_counter, gyro_address, vibration_counter;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int acc_axis[4], gyro_axis[4];
double gyro_pitch, gyro_roll, gyro_yaw;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int cal_int;
double gyro_axis_cal[4];

//Rotina de configuração
void setup(){
  Serial.begin(57600);                                                                  //Start the serial port.
  Wire.begin();                                                                         //Start the wire library as master
  TWBR = 12;                                                                            //Set the I2C clock speed to 400kHz.

  //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                                                    //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00010000;                                                                    //Configure digital poort 12 as output.

  PCICR |= (1 << PCIE0);                                                                // set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                              // set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                              // set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                              // set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                              // set PCINT3 (digital input 11)to trigger an interrupt on state change.

  for(data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);               //Read EEPROM for faster data access

  gyro_address = eeprom_data[32];                                                       //Store the gyro address in the variable.

  set_gyro_registers();                                                                 //Set the specific gyro registers.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(500);                                                                         //Wait for 500ms.
    digitalWrite(12, !digitalRead(12));                                                 //Change the led status to indicate error.
  }
  wait_for_receiver();                                                                  //Wait until the receiver is active.
  zero_timer = micros();                                                                //Set the zero_timer for the first loop.

  Serial.println("			Lista de comandos			");
  Serial.println(" Retire as hélices, escreva o cracatere e envie no campo serial");
  Serial.println("r = Mostrar os valores adquiridos pelo receptor");
  Serial.println("a = Mostrar os valores convertidos da IMU	");
  Serial.println("1 = Checar a rotação e vibração do motor 1  (frontal direito)");
  Serial.println("2 = Checar a rotação e vibração do motor 2  (traseiro direito)");
  Serial.println("3 = Checar a rotação e vibração do motor 3  (traseiro esquerdo)");
  Serial.println("4 = Checar a rotação e vibração do motor 4  (frontal esquerdo) ");
  Serial.println("5 = Checar vibração de todos os motores simultaneamente	");

  while(Serial.available())data = Serial.read();                                        //Empty the serial buffer.
  data = 0;                                                                             //Set the data variable back to zero.

}

//Main program loop
void loop(){
  while(zero_timer + 4000 > micros());                                                  //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                                                //Reset the zero timer.

  if(Serial.available() > 0){
    data = Serial.read();                                                               //Read the incomming byte.
    delay(100);                                                                         //Wait for any other bytes to come in
    while(Serial.available() > 0)loop_counter = Serial.read();                          //Empty the Serial buffer.
    new_function_request = true;                                                        //Set the new request flag.
    loop_counter = 0;                                                                   //Reset the loop_counter variable.
    cal_int = 0;                                                                        //Reset the cal_int variable to undo the calibration.
    start = 0;                                                                          //Set start to 0.
    first_angle = false;                                                                //Set first_angle to false.
    //Confirm the choice on the serial monitor.
    if(data == '2')Serial.println("Teste motor 2 (traseiro direito)");
    if(data == '3')Serial.println("Teste motor 3 (traseiro esquerdo)");
    if(data == '4')Serial.println("Teste motor 4 (frontal esquerdo)");
    if(data == '5')Serial.println("Testee todos os motores simultaneamente");

    // O pulso de 1500 irá armar os ESCs (BIDIRECIONAIS) e deixar os motores parados
    for(vibration_counter = 0; vibration_counter < 625; vibration_counter++){           //Do this loop 625 times
      delay(3);                                                                         //Wait 3000us.
      esc_1 = 1500;                                                                     //Pulso de 1500us para o ESC1 (parar motor)
      esc_2 = 1500;                                                                     //Pulso de 1500us para o ESC2 (parar motor)
      esc_3 = 1500;                                                                     //Pulso de 1500us para o ESC3 (parar motor).
      esc_4 = 1500;                                                                     //Pulso de 1500us para o ESC4 (parar motor)
      esc_pulse_output();                                                               //Envia os pulsos para os ESCs
    }
    vibration_counter = 0;                                                              //Reseta a contagem de vibrações
  }

  receiver_input_channel_3 = convert_receiver_channel(3);                               //Converte os valores recebidos pelo receptor para a escala de 1500-2000us (escala para 1 direção do ESC bidirecional)
  if(receiver_input_channel_3 < 1520)new_function_request = false;                      //A flag fica false quando o throttle está na menor posição ESC BIDIRECIONAL


  ////////////////////////////////////////////////////////////////////////////////////////////
  //Teste dos ESCs
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 0 && new_function_request == false){                                       //Apenas inicia o modo de teste na primeira vez de rodar o programa
    receiver_input_channel_3 = convert_receiver_channel(3);                             //Converte os valores recebidos pelo receptor para a escala de 1500-2000us (escala para 1 direção do ESC bidirecional)
    esc_1 = receiver_input_channel_3;                                                   //Largura de pulso do motor 1 igual ao valor do trhottle
    esc_2 = receiver_input_channel_3;                                                   //Largura de pulso do motor 1 igual ao valor do trhottle
    esc_3 = receiver_input_channel_3;                                                   //Largura de pulso do motor 1 igual ao valor do trhottle
    esc_4 = receiver_input_channel_3;                                                   //Largura de pulso do motor 1 igual ao valor do trhottle
    esc_pulse_output();                                                                 //Envia os pulsos para os ESCs

  ////////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'r' print the receiver signals.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'r'){
	Serial.println("Lendo valores do receptor");
    loop_counter ++;                                                                    //Increase the loop_counter variable.
    receiver_input_channel_1 = convert_receiver_channel(1);                           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2);                           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4);                           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    if(loop_counter == 125){                                                            //Print the receiver values when the loop_counter variable equals 250.
	Serial.println("Debug numero: %d"), debug; debug++;
      print_signals();                                                                  //Print the receiver values on the serial monitor.
      loop_counter = 0;                                                                 //Reset the loop_counter variable.
    }

    //Para ativar os motores:   start: |0 = parado|1 = preparando|2 = armado|
	//Passo 1: coloque o throttle no nível BAIXO (1500 uS) e YAW para a esquerda (1500 uS).
    if(receiver_input_channel_3 < 1550 && receiver_input_channel_4 < 1550)start = 1;
    //Passo 2: Coloque o Yaw na posição centro (1750 uS)
    if(start == 1 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1700)start = 2;
    //para parar os motores:
	//Coloque o throttle em nivel BAIXO (1500 uS) e YAW par a direita (2000 uS)
    if(start == 2 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1950)start = 0;

    esc_1 = 1500;                                                                       //Set the pulse for ESC 1 to 1500us.
    esc_2 = 1500;                                                                       //Set the pulse for ESC 2 to 1500us.
    esc_3 = 1500;                                                                       //Set the pulse for ESC 3 to 15000us.
    esc_4 = 1500;                                                                       //Set the pulse for ESC 4 to 1500us.
    esc_pulse_output();                                                                 //Send the ESC control pulses.
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //Teste individual ou em conjunto dos motores (1,2,3,4 ou 5(conjunto))
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){          //Dado inserido
    loop_counter ++;                                                                    //Printa os valores quando o chega a 250 loops
    if(new_function_request == true && loop_counter == 250){                            //Espera o usuário colocar o throttle em BAIXO	
      Serial.print("Coloca o trhottle no BAIXO (1500 uS). Atualmente está em: ");       //Aviso
      Serial.print(receiver_input_channel_3);                                           //Imprime a posição atual (bruta)
	  Serial.println("uS");
      loop_counter = 0;                                                                 //Reseta o contador
    }
    if(new_function_request == false){                                                  //Quando o throttle tiver na menor posição
      receiver_input_channel_3 = convert_receiver_channel(3);                           //Escalona o sinal do throttle para (1500 a 2000)
      if(data == '1' || data == '5')esc_1 = receiver_input_channel_3;                   //Se o teste é o motor 1, ou em todos, envia um pulso com o valor escalonado
      else esc_1 = 1500;                                                                //Desliga o motor caso não esteja sendo usado (1500 uS)
      if(data == '2' || data == '5')esc_2 = receiver_input_channel_3;                   //Se o teste é o motor 2, ou em todos, envia um pulso com o valor escalonado
      else esc_2 = 1500;                                                                //Desliga o motor caso não esteja sendo usado (1500 uS)
      if(data == '3' || data == '5')esc_3 = receiver_input_channel_3;                   //Se o teste é o motor 3, ou em todos, envia um pulso com o valor escalonado
      else esc_3 = 1500;                                                                //Desliga o motor caso não esteja sendo usado (1500 uS)
      if(data == '4' || data == '5')esc_4 = receiver_input_channel_3;                   //Se o teste é o motor 4, ou em todos, envia um pulso com o valor escalonado
      else esc_4 = 1500;                                                                //Desliga o motor caso não esteja sendo usado (1500 uS)

      esc_pulse_output();                                                               //Envia os valores de largura de puso para os ESCs

      //For balancing the propellors it's possible to use the accelerometer to measure the vibrations.
      if(eeprom_data[31] == 1){                                                         //The MPU-6050 is installed
        Wire.beginTransmission(gyro_address);                                           //Start communication with the gyro.
        Wire.write(0x3B);                                                               //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();                                                         //End the transmission.
        Wire.requestFrom(gyro_address,6);                                               //Request 6 bytes from the gyro.
        while(Wire.available() < 6);                                                    //Wait until the 6 bytes are received.
        acc_x = Wire.read()<<8|Wire.read();                                             //Add the low and high byte to the acc_x variable.
        acc_y = Wire.read()<<8|Wire.read();                                             //Add the low and high byte to the acc_y variable.
        acc_z = Wire.read()<<8|Wire.read();                                             //Add the low and high byte to the acc_z variable.

        acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));          //Cálculo do valor total mais recente do acelerômetro

        acc_av_vector = acc_total_vector[0];                                            //Atualiza o valor mais recente do acelerômetro

        for(start = 16; start > 0; start--){                                            //Loop para criar a MÉDIA MÓVEL dos valores totais do acelerômetro
          acc_av_vector += acc_total_vector[start];                                     //Adiciona o valor deslocado anteriormente
          acc_total_vector[start] = acc_total_vector[start - 1];                        //Desloca todos os valores para preencher o vetor
        }

        acc_av_vector /= 17;                                                            //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.

        if(vibration_counter < 20){                                                     //If the vibration_counter is less than 20 do this.
          vibration_counter ++;                                                         //Increment the vibration_counter variable.
          vibration_total_result += abs(acc_total_vector[0] - acc_av_vector);           //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
        }
        else{
          vibration_counter = 0;                                                        //If the vibration_counter is equal or larger than 20 do this.
          Serial.println(vibration_total_result/50);                                    //Print the total accelerometer vector divided by 50 on the serial monitor.
          vibration_total_result = 0;                                                   //Reset the vibration_total_result variable.
        }
      }
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'a' display the quadcopter angles.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'a'){
	Serial.println("Vizualizar valores dos ângulos adquiridos");
	Serial.println("Calbração do giroscópio (Não mecha o drone).");
    if(cal_int != 2000){
      Serial.print("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < 2000 ; cal_int ++){                                   //Take 2000 readings for calibration.
        if(cal_int % 125 == 0){
          digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
          Serial.print(".");
        }
        gyro_signalen();                                                                //Read the gyro output.
        gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
        //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
        PORTD |= B11110000;                                                             //Set digital poort 4, 5, 6 and 7 high.
		delayMicroseconds(1500);                                                        //Esperar 1500 us
        PORTD &= B00001111;                                                             //Set digital poort 4, 5, 6 and 7 low.
        delay(3);                                                                       //Wait 3 milliseconds before the next loop.
      }
      Serial.println(".");
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
      gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
      gyro_axis_cal[3] /= 2000;                                                         //Divide the yaw total by 2000.
    }
    else{
      ///We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
      PORTD |= B11110000;                                                               //Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(1500);                                                          //Esperar 1500 us
      PORTD &= B00001111;                                                               //Set digital poort 4, 5, 6 and 7 low.

      //Let's get the current gyro data.
      gyro_signalen();

      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += gyro_pitch * 0.0000611;                                           //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += gyro_roll * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the pitch angle to the roll angel.

      //Accelerometer angle calculations
      acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));           //Calculate the total accelerometer vector.

      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      angle_pitch_acc = asin((float)acc_y/acc_total_vector[0])* 57.296;                //Calculate the pitch angle.
      angle_roll_acc = asin((float)acc_x/acc_total_vector[0])* -57.296;                //Calculate the roll angle.
      
      if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
        angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }

      //We can't print all the data at once. This takes to long and the angular readings will be off.
      if(loop_counter == 0)Serial.print("Pitch: ");
      if(loop_counter == 1)Serial.print(angle_pitch ,0);
      if(loop_counter == 2)Serial.print(" Roll: ");
      if(loop_counter == 3)Serial.print(angle_roll ,0);
      if(loop_counter == 4)Serial.print(" Yaw: ");
      if(loop_counter == 5)Serial.println(gyro_yaw / 65.5 ,0);

      loop_counter ++;
      if(loop_counter == 60)loop_counter = 0;      
    }
  }
}
}


//This routine is called every time input 8, 9, 10 or 11 changed state.
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                      //Remember current input state.
      timer_1 = current_time;                                  //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                        //Remember current input state.
    receiver_input[1] = current_time - timer_1;                 //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                      //Remember current input state.
      timer_2 = current_time;                                  //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                        //Remember current input state.
    receiver_input[2] = current_time - timer_2;                //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                      //Remember current input state.
      timer_3 = current_time;                                  //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                        //Remember current input state.
    receiver_input[3] = current_time - timer_3;                //Channel 3 is current_time - timer_3.
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                      //Remember current input state.
      timer_4 = current_time;                                  //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                        //Remember current input state.
    receiver_input[4] = current_time - timer_4;                //Channel 4 is current_time - timer_4.
  }
}

//Checar se os valores brutos do receptor RC são validos (bidirecional na configuração unidirecional)
void wait_for_receiver(){
  byte zero = 0;                                                                //Zera os bits
  while(zero < 15){                                                             //Prende o programa enquanto zero nao tiver todos os 4 bits em 1
    if(receiver_input[1] < 2100 && receiver_input[1] > 1400)zero |= 0b00000001;  //Ativa o bit 0 caso o pulso do canal 1 esteja na faixa de 1400 a 2000 us
    if(receiver_input[2] < 2100 && receiver_input[2] > 1400)zero |= 0b00000010;  //Ativa o bit 1 caso o pulso do canal 2 esteja na faixa de 1400 a 2000 us
    if(receiver_input[3] < 2100 && receiver_input[3] > 1400)zero |= 0b00000100;  //Ativa o bit 2 caso o pulso do canal 3 esteja na faixa de 1400 a 2000 us
    if(receiver_input[4] < 2100 && receiver_input[4] > 1400)zero |= 0b00001000;  //Ativa o bit 3 caso o pulso do canal 4 esteja na faixa de 1400 a 2000 us
    delay(500);                                                                 //Espera 500 ms
  }
}

//Essa função garante que que os sinais brutos do receptor serão escalonados para a faixa de 1500-2000 uS (configuração unidirecional)
//**********Esta faixa é apenas para ESC's BIDIRECIONAIS. ESCs unidirecionais não serão armados ou o motor não consigirá parar************
//Os dados armazenados na memória EEPROM (baixo, medio e alto) calibrados são utilizados aqui
int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1750 + difference/2;                                  //If the channel is reversed
    else return 1750 - difference/2;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1750 + difference/2;                                  //If the channel is reversed
    else return 1750 + difference/2;                                             //If the channel is not reversed
  }
  else return 1750;
}

void print_signals(){													//Mostra a direção que o drone deve ir baseado nos sinais adquiridos
  Serial.print("Start:");												//Situação atual do motor
  Serial.print(start);

  Serial.print("  Roll:");
  if(receiver_input_channel_1 - 1730 < 0)Serial.print("<<<");
  else if(receiver_input_channel_1 - 1770 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Pitch:");
  if(receiver_input_channel_2 - 1730 < 0)Serial.print("^^^");
  else if(receiver_input_channel_2 - 1770 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Throttle:");
  if(receiver_input_channel_3 - 1730 < 0)Serial.print("vvv");
  else if(receiver_input_channel_3 - 1770 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if(receiver_input_channel_4 - 1730 < 0)Serial.print("<<<");
  else if(receiver_input_channel_4 - 1770 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}

void esc_pulse_output(){
  zero_timer = micros();
  PORTD |= B11110000;                                            //Set port 4, 5, 6 and 7 high at once
  timer_channel_1 = esc_1 + zero_timer;                          //Calculate the time when digital port 4 is set low.
  timer_channel_2 = esc_2 + zero_timer;                          //Calculate the time when digital port 5 is set low.
  timer_channel_3 = esc_3 + zero_timer;                          //Calculate the time when digital port 6 is set low.
  timer_channel_4 = esc_4 + zero_timer;                          //Calculate the time when digital port 7 is set low.

  while(PORTD >= 16){                                            //Execute the loop until digital port 4 to 7 is low.
    esc_loop_timer = micros();                                   //Check the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;     //When the delay time is expired, digital port 4 is set low.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;     //When the delay time is expired, digital port 5 is set low.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;     //When the delay time is expired, digital port 6 is set low.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;     //When the delay time is expired, digital port 7 is set low.
  }
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                      //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
    Wire.write(0x1B);                                            //Start reading @ register 0x1B
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                     //Check if the value is 0x08
      digitalWrite(12,HIGH);                                     //Turn on the warning led
      while(1)delay(10);                                         //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
    Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                      //End the transmission with the gyro    

  }  
}

void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro.
    Wire.write(0x3B);                                            //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                      //End the transmission.
    Wire.requestFrom(gyro_address,14);                           //Request 14 bytes from the gyro.
    while(Wire.available() < 14);                                //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];           //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;               //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];          //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;              //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];            //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                   //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                   //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                   //Invert acc_z if the MSB of EEPROM bit 30 is set.
}
