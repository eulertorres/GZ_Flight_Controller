/////CALIBRAÇÃO INICIAL (EEPROM) DO DRONE CRONUS [ARDUINO] YOUTUBE: GRAVIDADE ZERO/////
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
//ESTE SOFTWARE APENAS FUNCIONA PARA ESCs UNIDIRECIONAIS E BIDIRECIONAIS, OU SEJA,	  |
//PODE SER CONFIGURADO TANTO PARA A FAIXA DE (1000~2000us) quando para (1500~2000us). |
//PORÉM, É NECESSÁRIO INFORMAR AOS PROGRAMAS "CALIBRAÇÃO_ESC" E "CONTROLADORA_DE_VOO" |
//QUAL DESAS FAIXAS ESTÁ SENDO UTILIZADA. CASO CONTRÁRIO PODE HAVER UMA FALHA NO	  |
//CONTROLE DO ESC E POTENCIALMENTE CAUSAR ACIDENTES OU PERDA TOTAL DO COMPONENTE	  |
//FIQUE ATENTO A ESTE DETALHE ANTES DE UTILIZAR O SOFTWARE.							  |
//-------------------------------------------------------------------------------------
//ESTE PROGRAMA FOI BASEADO NO TRABALHO PRÉVIO DE JOOP BROOKING, SENDO TRADUZIDO E	  |
//ADAPTADO PARA O PROJETO CRONUS													  |
//FONTE: http://www.brokking.net/ymfc-al_main.html									  |
//AGRADECIMENTOS AO JOOP BROOKING POR CONTRIBUIR COM A DISSEMINAÇÃO DE CONHECIMENTOS  |
//SOBRE O UNIVERSO DOS VANTS. CANAL: https://www.youtube.com/user/MacPuffdog/featured |
//-------------------------------------------------------------------------------------

#include <Wire.h>               //Biblioteca responsável pela comunicação I²C
#include <EEPROM.h>             //Biblioteca resposável por armazenar informações na memória do Arduino

//Declaring Global Variables
byte antigo_canal_1, antigo_canal_2, antigo_canal_3, antigo_canal_4;
byte lowByte, highByte, type, giroscopio_address, error, clockspeed_ok;
byte channel_1_assign, channel_2_assign, channel_3_assign, channel_4_assign;
byte roll_axis, pitch_axis, yaw_axis;
byte receiver_check_byte, giroscopio_check_byte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
int address, cal_int;
unsigned long timer, timer_1, timer_2, timer_3, timer_4, current_time;
float giroscopio_pitch, giroscopio_roll, giroscopio_yaw;
float giroscopio_roll_cal, giroscopio_pitch_cal, giroscopio_yaw_cal;


//Rotina de Setup
//--------------------------------------------------------------------------------------------------
void setup(){
  pinMode(12, OUTPUT);		// Pino 12 como output para a LED (O resto é input automaticamente)
  PCICR |= (1 << PCIE0);    // registrador PCIE0 em ALTO, para habilitar scaneamento do PCMSK0 (interrupção)
  PCMSK0 |= (1 << PCINT0);  // PCINT0 em HIGH (GPIO 8) para acionar interrupção em qualquer mudança de nivel logico
  PCMSK0 |= (1 << PCINT1);  // PCINT1 em HIGH (GPIO 9) para acionar interrupção em qualquer mudança de nivel logico
  PCMSK0 |= (1 << PCINT2);  // PCINT2 em HIGH (GPIO 10) para acionar interrupção em qualquer mudança de nivel logico
  PCMSK0 |= (1 << PCINT3);  // PCINT3 em HIGH (GPIO 11) para acionar interrupção em qualquer mudança de nivel logico
  Wire.begin();             // Inicia comunicação I²C como mestre
  Serial.begin(57600);      // Conexão serial com 57600 bps
  delay(250);               // Tempo extra pro giroscopio iniciar
}

//Rotina principal
//--------------------------------------------------------------------------------------------------
void loop(){
  //Introdução do programa adaptado
  intro();
  
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("			Checagem do sistema"));
  Serial.println(F("==================================================="));
  delay(300);
  Serial.println(F("Checagem da velocidade do clock para I2C (400kHz)"));
  delay(300);
  
  TWBR = 12;                      // Velocidade do clock i2c para 400kHz.
  
  #if F_CPU == 16000000L          // Se o clock do CPU for 16MHz inclui a seguinte linha na compliação
    clockspeed_ok = 1;            // clockspeed_ok em ALTO
  #endif                          

  if(TWBR == 12 && clockspeed_ok){
    Serial.println(F("O clock i2c está configurado corretamente"));
  }
  else{
    Serial.println(F("Houve um problema na checagem, verifique as especificações da placa utilizada. (ERROR 8)"));
    error = 1;
  }
  
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("			  Configuração do transmissor RC"));
    Serial.println(F("==================================================="));
    delay(1000);
    Serial.print(F("Checagem para sinais válidos"));
    //Espera 10 segundos. Tempo limite para detectar sinais válidos do controle remoto (1000-2000us) de largura de pulso
    wait_for_receiver();
    Serial.println(F(""));
  }
  //Quit the program in case of an error
  if(error == 0){
    delay(2000);
    Serial.println(F("Coloque os manches na posição central do controle nos próximos 10 segundos"));
    for(int i = 9;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.println(" ");
    //Define a posição de CENTRO para este controle remoto
    center_channel_1 = receiver_input_channel_1;
    center_channel_2 = receiver_input_channel_2;
    center_channel_3 = receiver_input_channel_3;
    center_channel_4 = receiver_input_channel_4;
    Serial.println(F(""));
    Serial.println(F("Posições de CENTRO armazenadas"));
    Serial.print(F("GPIO 08 = "));
    Serial.println(receiver_input_channel_1);
    Serial.print(F("GPIO 09 = "));
    Serial.println(receiver_input_channel_2);
    Serial.print(F("GPIO 10 = "));
    Serial.println(receiver_input_channel_3);
    Serial.print(F("GPIO 11 = "));
    Serial.println(receiver_input_channel_4);
    Serial.println(F(""));
    Serial.println(F(""));
  }
  if(error == 0){  
    Serial.println(F("Movimente o manche do throttle para a maior posição, depois para o centro"));
    //checagem do movimento de trhottle
    check_receiver_inputs(1);
    Serial.print(F("Throttle está conectado na GPIO "));
    Serial.println((channel_3_assign & 0b00000111) + 7);
    if(channel_3_assign & 0b10000000)Serial.println(F("O canal ESTÁ invertido"));
    else Serial.println(F("O canal NÃO está invertido"));
    wait_sticks_zero();
    
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Mova o manche do Roll para simular o drone rolando para a direita (horário) e coloque no centro novamente"));
    //Checagem do movimento de Roll
    check_receiver_inputs(2);
    Serial.print(F("Roll está conectado ao GPIO "));
    Serial.println((channel_1_assign & 0b00000111) + 7);
    if(channel_1_assign & 0b10000000)Serial.println(F("O canal ESTÁ invertido"));
    else Serial.println(F("O canal NÃO está invertido"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move o manche do Pitch para simular frente do drone levantando, depois para o centro novamente"));
    //Checagem do movimento de Pitch 
    check_receiver_inputs(3);
    Serial.print(F("Pitch está conectado ao GPIO "));
    Serial.println((channel_2_assign & 0b00000111) + 7);
    if(channel_2_assign & 0b10000000)Serial.println(F("O canal ESTÁ invertido"));
    else Serial.println(F("O canal NÃO está invertido"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Mova o manche do Yaw para simular o drone virando para a direita, depois para o centro novamente"));
    //Checagem do movimento de Yaw
    check_receiver_inputs(4);
    Serial.print(F("Yaw está conectado ao GPIO "));
    Serial.println((channel_4_assign & 0b00000111) + 7);
    if(channel_4_assign & 0b10000000)Serial.println(F("O canal ESTÁ invertido"));
    else Serial.println(F("O canal NÃO está invertido"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Mova lentamente os manches para cada extremidade"));
    Serial.println(F("Quando estiver pronto, coloque todos no centro novamente"));
    //Register the min and max values of the receiver channels
    register_min_max();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Valores finais de ALTO, CENTRO, BAIXO detectados na configuração"));
    Serial.print(F("GPIO 08:"));
    Serial.print(low_channel_1);
    Serial.print(F(" - "));
    Serial.print(center_channel_1);
    Serial.print(F(" - "));
    Serial.println(high_channel_1);
    Serial.print(F("GPIO 09:"));
    Serial.print(low_channel_2);
    Serial.print(F(" - "));
    Serial.print(center_channel_2);
    Serial.print(F(" - "));
    Serial.println(high_channel_2);
    Serial.print(F("GPIO 10:"));
    Serial.print(low_channel_3);
    Serial.print(F(" - "));
    Serial.print(center_channel_3);
    Serial.print(F(" - "));
    Serial.println(high_channel_3);
    Serial.print(F("GPIO 11:"));
    Serial.print(low_channel_4);
    Serial.print(F(" - "));
    Serial.print(center_channel_4);
    Serial.print(F(" - "));
    Serial.println(high_channel_4);
    Serial.println(F("Simule o movimento ''frente do drone para cima'' para continuar"));
    check_to_continue();
  }
    
  if(error == 0){
    //Qual IMU está sendo utilizada
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("			Identificação do giroscopio"));
    Serial.println(F("==================================================="));
    delay(2000);
    
    Serial.println(F("Procurando pela IMU MPU-6050 no endereço 0x68/104"));
    delay(1000);
    if(search_giroscopio(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 encontrado no endereço 0x68"));
      type = 1;
      giroscopio_address = 0x68;
    }
    
    if(type == 0){
      Serial.println(F("Procurando pela IMU MPU-6050 no endereço 0x69/105"));
      delay(1000);
      if(search_giroscopio(0x69, 0x75) == 0x68){
        Serial.println(F("MPU-6050 encontrado no endereço 0x69"));
        type = 1;
        giroscopio_address = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("Procurando pela IMU L3G4200D no endereço 0x68/104"));
      delay(1000);
      if(search_giroscopio(0x68, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D encontrado no endereço 0x68"));
        type = 2;
        giroscopio_address = 0x68;
      }
    }
    
    if(type == 0){
      Serial.println(F("Procurando pela IMU L3G4200D no endereço 0x69/105"));
      delay(1000);
      if(search_giroscopio(0x69, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D encontrado no endereço 0x69"));
        type = 2;
        giroscopio_address = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("Procurando pela IMU L3GD20H no endereço 0x6A/106"));
      delay(1000);
      if(search_giroscopio(0x6A, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H encontrado no endereço 0x6A"));
        type = 3;
        giroscopio_address = 0x6A;
      }
    }
    
    if(type == 0){
     Serial.println(F("Procurando pela IMU L3GD20H no endereço 0x6B/107"));
      delay(1000);
      if(search_giroscopio(0x6B, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H encontrado no endereço 0x6B"));
        type = 3;
        giroscopio_address = 0x6B;
      }
    }
    
    if(type == 0){
      Serial.println(F("Nenhum giroscopio encontrado!!! (ERROR 3)"));
	  Serial.println(F("Verifique a conexão da IMU com o Arduino (SCL e SDA)"));
      error = 1;
    }
    
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("			 Configuração da IMU"));
      Serial.println(F("==================================================="));
      start_giroscopio(); //Configuração
    }
  }
  
  //Se o giroscopio foi encontrado podemos prosseguir para a calibração
  if(error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("				Calibração do giroscopio"));
    Serial.println(F("==================================================="));
    Serial.println(F("Não mecha o drone!! A calibração começa em 5 segundos"));
    delay(5000);
    Serial.println(F("Calibrando o giroscopio, isso demora +/- 8 seconds"));
    Serial.print(F("Por favor espera com o drone parado"));
    //Múltiplas medidas para identificar a média do erro de cada um dos eixos
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      giroscopio_signalen();                                           //Read the giroscopio output.
      giroscopio_roll_cal += giroscopio_roll;                                //Ad roll value to giroscopio_roll_cal.
      giroscopio_pitch_cal += giroscopio_pitch;                              //Ad pitch value to giroscopio_pitch_cal.
      giroscopio_yaw_cal += giroscopio_yaw;                                  //Ad yaw value to giroscopio_yaw_cal.
      delay(4);                                                  //Wait 3 milliseconds before the next loop.
    }
    //Now that we have 2000 measures, we need to devide by 2000 to get the average giroscopio offset.
    giroscopio_roll_cal /= 2000;                                       //Divide the roll total by 2000.
    giroscopio_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
    giroscopio_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(giroscopio_roll_cal);
    Serial.print(F("Axis 2 offset="));
    Serial.println(giroscopio_pitch_cal);
    Serial.print(F("Axis 3 offset="));
    Serial.println(giroscopio_yaw_cal);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("giroscopio axes configuration"));
    Serial.println(F("==================================================="));
    
    //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_giroscopio_axes(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(roll_axis & 0b00000011);
      if(roll_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_giroscopio_axes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_giroscopio_axes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
    }
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("LED test"));
    Serial.println(F("==================================================="));
    digitalWrite(12, HIGH);
    Serial.println(F("The LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
    digitalWrite(12, LOW);
  }
  
  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("==================================================="));
    Serial.println(F("Final setup check"));
    Serial.println(F("==================================================="));
    delay(1000);
    if(receiver_check_byte == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }
    delay(1000);
    if(giroscopio_check_byte == 0b00000111){
      Serial.println(F("giroscopio axes ok"));
    }
    else{
      Serial.println(F("giroscopio exes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }     
  
  if(error == 0){
    //If all is good, store the information in the EEPROM
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));
    EEPROM.write(0, center_channel_1 & 0b11111111);
    EEPROM.write(1, center_channel_1 >> 8);
    EEPROM.write(2, center_channel_2 & 0b11111111);
    EEPROM.write(3, center_channel_2 >> 8);
    EEPROM.write(4, center_channel_3 & 0b11111111);
    EEPROM.write(5, center_channel_3 >> 8);
    EEPROM.write(6, center_channel_4 & 0b11111111);
    EEPROM.write(7, center_channel_4 >> 8);
    EEPROM.write(8, high_channel_1 & 0b11111111);
    EEPROM.write(9, high_channel_1 >> 8);
    EEPROM.write(10, high_channel_2 & 0b11111111);
    EEPROM.write(11, high_channel_2 >> 8);
    EEPROM.write(12, high_channel_3 & 0b11111111);
    EEPROM.write(13, high_channel_3 >> 8);
    EEPROM.write(14, high_channel_4 & 0b11111111);
    EEPROM.write(15, high_channel_4 >> 8);
    EEPROM.write(16, low_channel_1 & 0b11111111);
    EEPROM.write(17, low_channel_1 >> 8);
    EEPROM.write(18, low_channel_2 & 0b11111111);
    EEPROM.write(19, low_channel_2 >> 8);
    EEPROM.write(20, low_channel_3 & 0b11111111);
    EEPROM.write(21, low_channel_3 >> 8);
    EEPROM.write(22, low_channel_4 & 0b11111111);
    EEPROM.write(23, low_channel_4 >> 8);
    EEPROM.write(24, channel_1_assign);
    EEPROM.write(25, channel_2_assign);
    EEPROM.write(26, channel_3_assign);
    EEPROM.write(27, channel_4_assign);
    EEPROM.write(28, roll_axis);
    EEPROM.write(29, pitch_axis);
    EEPROM.write(30, yaw_axis);
    EEPROM.write(31, type);
    EEPROM.write(32, giroscopio_address);
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
        
    
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel_1_assign != EEPROM.read(24))error = 1;
    if(channel_2_assign != EEPROM.read(25))error = 1;
    if(channel_3_assign != EEPROM.read(26))error = 1;
    if(channel_4_assign != EEPROM.read(27))error = 1;
    
    if(roll_axis != EEPROM.read(28))error = 1;
    if(pitch_axis != EEPROM.read(29))error = 1;
    if(yaw_axis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(giroscopio_address != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
  }
  else{
   Serial.println(F("The setup is aborted due to an error."));
   Serial.println(F("Check the Q and A page of the YMFC-AL project on:"));
   Serial.println(F("www.brokking.net for more information about this error."));
  }
  while(1);
}

//Search for the giroscopio and check the Who_am_I register
byte search_giroscopio(int giroscopio_address, int who_am_i){
  Wire.beginTransmission(giroscopio_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(giroscopio_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = giroscopio_address;
  return lowByte;
}

void start_giroscopio(){
  //Setup the L3G4200D or L3GD20H
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                             //Start communication with the giroscopio with the address encontrado during search
    Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
    Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the giroscopio and enable all axis)
    Wire.endTransmission();                                      //End the transmission with the giroscopio

    Wire.beginTransmission(address);                             //Start communication with the giroscopio (adress 1101001)
    Wire.write(0x20);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 6 bytes from the giroscopio
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x20 is set to:"));
    Serial.println(Wire.read(),BIN);

    Wire.beginTransmission(address);                             //Start communication with the giroscopio  with the address encontrado during search
    Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
    Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the giroscopio
    
    Wire.beginTransmission(address);                             //Start communication with the giroscopio (adress 1101001)
    Wire.write(0x23);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 6 bytes from the giroscopio
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x23 is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the giroscopio
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the giroscopio
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the giroscopio
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the giroscopio
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the giroscopio
    Wire.write(0x1B);                                            //giroscopio_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the giroscopio (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the giroscopio
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void giroscopio_signalen(){
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                             //Start communication with the giroscopio
    Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 6);                                //Request 6 bytes from the giroscopio
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    giroscopio_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(cal_int == 2000)giroscopio_roll -= giroscopio_roll_cal;               //Only compensate after the calibration
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    giroscopio_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(cal_int == 2000)giroscopio_pitch -= giroscopio_pitch_cal;             //Only compensate after the calibration
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    giroscopio_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(cal_int == 2000)giroscopio_yaw -= giroscopio_yaw_cal;                 //Only compensate after the calibration
  }
  if(type == 1){
    Wire.beginTransmission(address);                             //Start communication with the giroscopio
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address,6);                                 //Request 6 bytes from the giroscopio
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    giroscopio_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(cal_int == 2000)giroscopio_roll -= giroscopio_roll_cal;               //Only compensate after the calibration
    giroscopio_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(cal_int == 2000)giroscopio_pitch -= giroscopio_pitch_cal;             //Only compensate after the calibration
    giroscopio_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(cal_int == 2000)giroscopio_yaw -= giroscopio_yaw_cal;                 //Only compensate after the calibration
  }
}

//Checa se o receptor obtem valores válidos nos próximos 30 segundos
void check_receiver_inputs(byte movement){
  byte trigger = 0;
  int pulse_length;
  timer = millis() + 30000;
  while(timer > millis() && trigger == 0){
    delay(250);
    if(receiver_input_channel_1 > 1750 || receiver_input_channel_1 < 1250){
      trigger = 1;
      receiver_check_byte |= 0b00000001;
      pulse_length = receiver_input_channel_1;
    }
    if(receiver_input_channel_2 > 1750 || receiver_input_channel_2 < 1250){
      trigger = 2;
      receiver_check_byte |= 0b00000010;
      pulse_length = receiver_input_channel_2;
    }
    if(receiver_input_channel_3 > 1750 || receiver_input_channel_3 < 1250){
      trigger = 3;
      receiver_check_byte |= 0b00000100;
      pulse_length = receiver_input_channel_3;
    }
    if(receiver_input_channel_4 > 1750 || receiver_input_channel_4 < 1250){
      trigger = 4;
      receiver_check_byte |= 0b00001000;
      pulse_length = receiver_input_channel_4;
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("Nenhum movimento detectado nos últimos 30 segundos :( (ERROR 2)"));
	Serial.println(F("Verifique a conexão do recepetor nos GPIOs corretor e a bindagem com o RC"));
  }
  //Assign the stick to the function.
  else{
    if(movement == 1){
      channel_3_assign = trigger;		// Throttle
      if(pulse_length < 1250)channel_3_assign += 0b10000000;
    }
    if(movement == 2){
      channel_1_assign = trigger;		// Roll
      if(pulse_length < 1250)channel_1_assign += 0b10000000;
    }
    if(movement == 3){
      channel_2_assign = trigger;		// Pitch
      if(pulse_length < 1250)channel_2_assign += 0b10000000;
    }
    if(movement == 4){
      channel_4_assign = trigger;		// Yaw
      if(pulse_length < 1250)channel_4_assign += 0b10000000;
    }
  }
}

void check_to_continue(){
  byte continue_byte = 0;
  while(continue_byte == 0){
    if(channel_2_assign == 0b00000001 && receiver_input_channel_1 > center_channel_1 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000001 && receiver_input_channel_1 < center_channel_1 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000010 && receiver_input_channel_2 > center_channel_2 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000010 && receiver_input_channel_2 < center_channel_2 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000011 && receiver_input_channel_3 > center_channel_3 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000011 && receiver_input_channel_3 < center_channel_3 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000100 && receiver_input_channel_4 > center_channel_4 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000100 && receiver_input_channel_4 < center_channel_4 - 150)continue_byte = 1;
    delay(100);
  }
  wait_sticks_zero();
}

//Check if the transmitter sticks are in the neutral position
void wait_sticks_zero(){
  byte zero = 0;
  while(zero < 15){
    if(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)zero |= 0b00000001;
    if(receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)zero |= 0b00000010;
    if(receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)zero |= 0b00000100;
    if(receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)zero |= 0b00001000;
    delay(100);
  }
}

//Checck if the receiver values are valid within 10 seconds
void wait_for_receiver(){
  byte zero = 0;
  timer = millis() + 10000;
  while(timer > millis() && zero < 15){
    if(receiver_input_channel_1 < 2100 && receiver_input_channel_1 > 900)zero |= 0b00000001;
    if(receiver_input_channel_2 < 2100 && receiver_input_channel_2 > 900)zero |= 0b00000010;
    if(receiver_input_channel_3 < 2100 && receiver_input_channel_3 > 900)zero |= 0b00000100;
    if(receiver_input_channel_4 < 2100 && receiver_input_channel_4 > 900)zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  if(zero == 0){
    error = 1;
    Serial.println(F("."));
    Serial.println(F("Nenhum sinal válido do receptor encontrado!!! (ERROR 1)"));
  }
  else Serial.println(F(" OK"));
}

//Register the min and max receiver values and exit when the sticks are back in the neutral position
void register_min_max(){
  byte zero = 0;
  low_channel_1 = receiver_input_channel_1;
  low_channel_2 = receiver_input_channel_2;
  low_channel_3 = receiver_input_channel_3;
  low_channel_4 = receiver_input_channel_4;
  while(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)delay(250);
  Serial.println(F("Measuring endpoints...."));
  while(zero < 15){
    if(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)zero |= 0b00000001;
    if(receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)zero |= 0b00000010;
    if(receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)zero |= 0b00000100;
    if(receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)zero |= 0b00001000;
    if(receiver_input_channel_1 < low_channel_1)low_channel_1 = receiver_input_channel_1;
    if(receiver_input_channel_2 < low_channel_2)low_channel_2 = receiver_input_channel_2;
    if(receiver_input_channel_3 < low_channel_3)low_channel_3 = receiver_input_channel_3;
    if(receiver_input_channel_4 < low_channel_4)low_channel_4 = receiver_input_channel_4;
    if(receiver_input_channel_1 > high_channel_1)high_channel_1 = receiver_input_channel_1;
    if(receiver_input_channel_2 > high_channel_2)high_channel_2 = receiver_input_channel_2;
    if(receiver_input_channel_3 > high_channel_3)high_channel_3 = receiver_input_channel_3;
    if(receiver_input_channel_4 > high_channel_4)high_channel_4 = receiver_input_channel_4;
    delay(100);
  }
}

//Check if the angular position of a giroscopio axis is changing within 10 seconds
void check_giroscopio_axes(byte movement){
  byte trigger_axis = 0;
  float giroscopio_angle_roll, giroscopio_angle_pitch, giroscopio_angle_yaw;
  //Reset all axes
  giroscopio_angle_roll = 0;
  giroscopio_angle_pitch = 0;
  giroscopio_angle_yaw = 0;
  giroscopio_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && giroscopio_angle_roll > -30 && giroscopio_angle_roll < 30 && giroscopio_angle_pitch > -30 && giroscopio_angle_pitch < 30 && giroscopio_angle_yaw > -30 && giroscopio_angle_yaw < 30){
    giroscopio_signalen();
    if(type == 2 || type == 3){
      giroscopio_angle_roll += giroscopio_roll * 0.00007;              //0.00007 = 17.5 (md/s) / 250(Hz)
      giroscopio_angle_pitch += giroscopio_pitch * 0.00007;
      giroscopio_angle_yaw += giroscopio_yaw * 0.00007;
    }
    if(type == 1){
      giroscopio_angle_roll += giroscopio_roll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
      giroscopio_angle_pitch += giroscopio_pitch * 0.0000611;
      giroscopio_angle_yaw += giroscopio_yaw * 0.0000611;
    }
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the giroscopio
  }
  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((giroscopio_angle_roll < -30 || giroscopio_angle_roll > 30) && giroscopio_angle_pitch > -30 && giroscopio_angle_pitch < 30 && giroscopio_angle_yaw > -30 && giroscopio_angle_yaw < 30){
    giroscopio_check_byte |= 0b00000001;
    if(giroscopio_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((giroscopio_angle_pitch < -30 || giroscopio_angle_pitch > 30) && giroscopio_angle_roll > -30 && giroscopio_angle_roll < 30 && giroscopio_angle_yaw > -30 && giroscopio_angle_yaw < 30){
    giroscopio_check_byte |= 0b00000010;
    if(giroscopio_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((giroscopio_angle_yaw < -30 || giroscopio_angle_yaw > 30) && giroscopio_angle_roll > -30 && giroscopio_angle_roll < 30 && giroscopio_angle_pitch > -30 && giroscopio_angle_pitch < 30){
    giroscopio_check_byte |= 0b00000100;
    if(giroscopio_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("Nenhum movimento angular foi detectado :( (ERROR 4)"));
	Serial.println(F("Verifique a conexão da IMU"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

//Toda vez que houver uma mudança nos pinos 8,9, 10 ou 11 o programa interrompe e executa isso:
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Caso a entrada 8 esteja em alto
    if(antigo_canal_1 == 0){                                   //Entrada 8 estava baixo, foi pra alto
      antigo_canal_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(antigo_canal_1 == 1){                                //Entrada 8 estava alto, foi pra baixo
    antigo_canal_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Caso a entrada 9 esteja em alto
    if(antigo_canal_2 == 0){                                   //Entrada 9 estava baixo, foi pra alto
      antigo_canal_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(antigo_canal_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    antigo_canal_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Caso a entrada 10 esteja em alto
    if(antigo_canal_3 == 0){                                   //Entrada 10 estava baixo, foi pra alto
      antigo_canal_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(antigo_canal_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    antigo_canal_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Caso a entrada 11 esteja em alto
    if(antigo_canal_4 == 0){                                   //Entrada 11 estava baixo, foi pra alto
      antigo_canal_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(antigo_canal_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    antigo_canal_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

//Intro subroutine
void intro(){
  Serial.println(F("==================================================="));
  delay(200);
  Serial.println(F(""));
  Serial.println(F("        Drone"));
  delay(100);
  Serial.print(F("  Cronus"));
  delay(100);
  Serial.print(F("  V1.0"));  
  delay(100);
  Serial.println(F("  - Arduino"));    
  delay(400);
  Serial.println(F(""));
  Serial.println(F("      Gravidade"));
  delay(100);
  Serial.print(F("      Controller"));
  delay(1000);
  Serial.println(F(""));
  Serial.println(F("  Programa de configuração da memória EEPROM"));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  delay(1500);
  Serial.println(F("Youtube: "));
  Serial.println(F(""));
  Serial.println(F("Tome cuidado e divirta-se :)"));
}
