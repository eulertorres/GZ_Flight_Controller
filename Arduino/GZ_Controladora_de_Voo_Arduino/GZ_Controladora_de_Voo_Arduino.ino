////////CONTROLADORA DE VOO PARA O DRONE ABELHINHA. YOUTUBE: GRAVIDADE ZERO////////////
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

#include <Wire.h>                          //Biblioteca para comunicação I²C com a MPU6050 e ADS1115
#include <EEPROM.h>                        //Biblioteca pro armazenamento na memória EEPROM do Arduino


//Ajuste de ganho do PID e Limites. É necessário ser feito manualmente
//---------------------------------------------------------------------------------------
float pid_p_gain_roll = 1.3;               //Ganho da parte P (Proporcional) do Roll			
float pid_i_gain_roll = 0.04;              //Ganho da parte I (Integrador) do Roll
float pid_d_gain_roll = 18.0;              //Ganho da parte D (Derivativo) do Roll
int pid_max_roll = 400;                    //Saída máxima e mínima do PID para o roll

float pid_p_gain_pitch = pid_p_gain_roll;  //Ganho da parte P (Proporcional) do Pitch
float pid_i_gain_pitch = pid_i_gain_roll;  //Ganho da parte I (Integrador) do Pitch
float pid_d_gain_pitch = pid_d_gain_roll;  //Ganho da parte D (Derivativo) do Pitch
int pid_max_pitch = pid_max_roll;          //Saída máxima e mínima do PID para o pitch

float pid_p_gain_yaw = 4.0;                //Ganho da parte P (Proporcional) do Pitch
float pid_i_gain_yaw = 0.02;               //Ganho da parte I (Integrador) do Pitch
float pid_d_gain_yaw = 0.0;                //Ganho da parte D (Derivativo) do Pitch
int pid_max_yaw = 400;                     //Saída máxima e mínima do PID para o pitch

boolean auto_level = true;                 //Balanceamento automático |||CUIDADO AO DESATIVAR|||

// Variáveis globais
//--------------------------------------------------------------------------------------------------------------------
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4; // Nível do sinal anterior do canal
byte eeprom_data[36];												 // Dados adquiridos na configuração inicial
byte highByte, lowByte;												 // Bytes comunicação I²C
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4; // Dados escalonados receptor
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter; 						 // Contador Largura de pulso dos canais
int esc_1, esc_2, esc_3, esc_4;										 // Valor final enviado aos ESCs para controle dos motores
int throttle, battery_voltage;										 // Valor da bateria adquirido pelo divisor de tensão
int cal_int, start, gyro_address;									 // Endereço da IMU
int receiver_input[5];												 // Dados brutos do receptor
int temperature;													 // Temperatura da IMU
int acc_axis[4], gyro_axis[4];										 // Buffer de dados acelerômetro e giroscópio
float roll_level_adjust, pitch_level_adjust;					     // Ajuste fino do Roll e Pitch

long acc_x, acc_y, acc_z, acc_total_vector;							 // Vetor da aceleração resultante
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;								 // Vetores de velocidade angular
double gyro_axis_cal[4];											 // Calibração do girosópio
float pid_error_temp;												 // Erro do PID
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

// Rotina de Setup
//--------------------------------------------------------------------------------------------------------------------
void setup(){
  //Serial.begin(57600);													// Habilitar comunicação Serial
  //Armazenamento dos dados da memória EEPROM adquiridos no programa de configuração inicial
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                //Reseta a variavel
  gyro_address = eeprom_data[32];                                           //Endereço da IMU

  Wire.begin();                                                             //Comunicação I²C como Mestre
  TWBR = 12;                                                                //registrador do clock I²C como 400Khz
  DDRD |= B11110000;                                                        //Portas 4, 5, 6 e 7 do arduino como saídas
  DDRB |= B00110000;                                                        //Portas 12 e 13 do arduino como saídas

  //LED para indicar inincio da configuração
  digitalWrite(12,HIGH);                                                    //Liga a led

  //Verificação de assinatura de Joop Brooking na memória EEPROM
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //Caso o drone não tenha IMU, interrompe o programa
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                     //Checagem e configuração dos registradores da IMU

  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Espera 5 segundos antes de continuar (tenpo te rotina )
    PORTD |= B11110000;                                                     //Porta 4, 5, 6 e 7 em nivel ALTO
    delayMicroseconds(1500);                                                //Espera 1.5 milisegundos (1500us).
    PORTD &= B00001111;                                                     //Porta 4, 5, 6 e 7 nivel BAIXO
    delayMicroseconds(3000);                                                //Espera 3 ms
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Realiza a leitura de 2000 amostras do giroscópio
    if(cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));                //Led pisca para indicar calibração
    gyro_signalen();                                                        //Leitura da saída da IMU
    gyro_axis_cal[1] += gyro_axis[1];                                       //Acumula o resultado do 1 byte (roll)
    gyro_axis_cal[2] += gyro_axis[2];                                       //Acumula o resultado do 2 byte (pitch)
    gyro_axis_cal[3] += gyro_axis[3];                                       //Acumula o resultado do 3 byte (yaw)
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Porta 4, 5, 6 e 7 em nivel ALTO
    delayMicroseconds(1500);                                                //Espera 1.5 milisegundos (1500us). (Desliga o motor)
    PORTD &= B00001111;                                                     //Porta 4, 5, 6 e 7 nivel BAIXO
    delay(3);                                                               //Espera 3 ms
  }
  //Adquirimos a média simples das 2000 amostras para estimar o erro do giroscópio
  gyro_axis_cal[1] /= 2000;                                                 //Divide o roll  por 2000
  gyro_axis_cal[3] /= 2000;                                                 //Divide o yaw   por 2000
  gyro_axis_cal[2] /= 2000;                                                 //Divide o pitch por 2000

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Registrador PCINT0 em alto (entrada digital 8)  para causar uma interrupção em qualquer mudança.
  PCMSK0 |= (1 << PCINT1);                                                  //Registrador PCINT1 em alto (entrada digital 9)  para causar uma interrupção em qualquer mudança.
  PCMSK0 |= (1 << PCINT2);                                                  //Registrador PCINT2 em alto (entrada digital 10) para causar uma interrupção em qualquer mudança.
  PCMSK0 |= (1 << PCINT3);                                                  //Registrador PCINT3 em alto (entrada digital 11) para causar uma interrupção em qualquer mudança.

  //Espera o throttle estar na menor posição (valor ja convertido em gyro_signalen()) e yaw no meio (impedir que drone realize algum movimento yaw)
  while(receiver_input_channel_3 < 1730 || receiver_input_channel_3 > 1770 || receiver_input_channel_4 < 1650){
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Escalonamento para a faixa de 1500 a 2000 us (throtle)
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Escalonamento para a faixa de 1500 a 2000 us (yaw)
    start ++;                                                               //Incrementa o contador para piscar a LED a cada 500 ms
    //Envia pulsos de 1500 us aos ESCs bidirecionais (para rotação)
    PORTD |= B11110000;                                                     //Porta 4, 5, 6 e 7 em nivel ALTO
    delayMicroseconds(1500);                                                //Espera 1.5 milisegundos (1500us) Para desligar os motores
    PORTD &= B00001111;                                                     //Porta 4, 5, 6 e 7 nivel BAIXO
    delay(3);                                                               //Espera 3 ms
    if(start == 125){                                                       //A cada 500 ms
      digitalWrite(12, !digitalRead(12));                                   //O led fica piscando
      start = 0;                                                            //Reseta o contador
    }
  }
  start = 0;                                                                //Reseta a variável

  //Cálculo da tensão atual da bateria (importante) [Divisor de tensão R1 = 1K e R2 = 350R]
  //O diodo apresenta queda de tensão equivalente a 65 (decimal)
  //A bateria cheia (12,6V) deve mostrar na porta analógica 3.3V (valor máximo da porta analógica do ESP32[projeto final])
  //12,6V deve ser lido como 1023 na porta analógica A0
  //1260 / 1023 = 1.2317. Cada valor incrementado representa 0,012 V da bateria (Resolução)
  //Exemplo: Caso seja lido 787 na porta A0, o valor de tensão da bateria será 10.5 V (tensão_bateria = 1050)
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  digitalWrite(12,LOW);                                                     //Desliga a LED
  loop_timer = micros();                                                    //Set the timer for the next loop.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }


  //Para iniciar os motores - Passo 1: Coloque o throttle na menor posição e Yaw para esquerda
  if(receiver_input_channel_3 < 1550 && receiver_input_channel_4 < 1550)start = 1;
  //Passo 2: Quando o Yaw voltar para o CENTRO
  if(start == 1 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1700){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Ângulo inicial do pitch será do acelerômetro (gravidade como referência)
    angle_roll = angle_roll_acc;                                            //Ângulo inicial do roll  será do acelerômetro (gravidade como referência)
    gyro_angles_set = true;                                                 //Flag para indicar inicio da IMU

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Para parar os motores: Throttle na menor posição e Yaw todo para direita
  if(start == 2 && receiver_input_channel_3 < 1550 && receiver_input_channel_4 > 1950)start = 0;

  //O ponto de referência do PID é em graus por segundo. O roll será a entrada deste sistema-------------------
  //Para converter a largura de pulso para graus por segundo, iremos dividir por 1.5. O ângulo maximo será então (250-4)/1.5 = 164 [graus/s]
  pid_roll_setpoint = 0;
  //O ponto de referência só é definido quando o canal é +/- 8 us diferente do CENTRO [Evitar flutuaçoes afetarem o PID]
  if(receiver_input_channel_1 > 1754)pid_roll_setpoint = receiver_input_channel_1 - 1754;
  else if(receiver_input_channel_1 < 1746)pid_roll_setpoint = receiver_input_channel_1 - 1746;
  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 1.5;                                                 //Converção para entrada do PID_roll ser em graus por segundo

  //Entrada do sistema de controle para o pitch -------------------------------------------------------------------------
  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1754)pid_pitch_setpoint = receiver_input_channel_2 - 1754;
  else if(receiver_input_channel_2 < 1746)pid_pitch_setpoint = receiver_input_channel_2 - 1746;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 1.5;                                                 //Converção para entrada do PID_pitch ser em graus por segundo

  //Entrada do sistema de controle para o Yaw -------------------------------------------------------------------------
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1525){ //Não rotacionar o drone quando o throttle tiver em BAIXO
    if(receiver_input_channel_4 > 1754)pid_yaw_setpoint = (receiver_input_channel_4 - 1754)/1.5;
    else if(receiver_input_channel_4 < 1746)pid_yaw_setpoint = (receiver_input_channel_4 - 1746)/1.5;
  }
  
  calculate_pid();                                                            //Com as entradas prontas para os PIDs, podemos calcular a saída

  //Compesação do descarga da bateria no contolador PID (com filtro complementar para reudizr o ruído da leitura analógica)
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH); // Aviso caso o nivel da bateria esteja muito baixo

  throttle = receiver_input_channel_3;                                      //O throttle será o sinal de controle principal

  if (start == 2){                                                          //Motores estão armados
    if (throttle > 1900) throttle = 1900;                                   //O PID precisa de uma parte da largura de pulso para atuar no drone
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Esc 1: Frontal -direito  - anti-horário	||								||
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Esc 2: Traseiro direito -  horário		|| IMPORTANTE SEGUIR ESSA ORDEM ||
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Esc 3: Traseiro esquerdo - anti-horário   ||								||
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Esc 4: Frontal  esquerdo - horário        ||								||

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Compensa a bateria apenas se tiver entre 8 e 12.4 Volts
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compoensação para o ESC 1 para a descarga da bateria
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compoensação para o ESC 2 para a descarga da bateria
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compoensação para o ESC 3 para a descarga da bateria
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compoensação para o ESC 4 para a descarga da bateria
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Os motores continuam rodando com o valor de throttle no baixo
    if (esc_2 < 1100) esc_2 = 1100;                                         //Os motores continuam rodando com o valor de throttle no baixo
    if (esc_3 < 1100) esc_3 = 1100;                                         //Os motores continuam rodando com o valor de throttle no baixo
    if (esc_4 < 1100) esc_4 = 1100;                                         //Os motores continuam rodando com o valor de throttle no baixo

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limite da largura de pulso deve ser 2000 us
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limite da largura de pulso deve ser 2000 us
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limite da largura de pulso deve ser 2000 us
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limite da largura de pulso deve ser 2000 us 
  }

  else{
    esc_1 = 1500;                                                           //Se não estiver armado, mantém o esc 1 no valor mínimo
    esc_2 = 1500;                                                           //Se não estiver armado, mantém o esc 2 no valor mínimo
    esc_3 = 1500;                                                           //Se não estiver armado, mantém o esc 3 no valor mínimo
    esc_4 = 1500;                                                           //Se não estiver armado, mantém o esc 4 no valor mínimo
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/fqEkVcqxtU8
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    
  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals. 
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //Termina a transmissão.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    receiver_input_channel_1 = convert_receiver_channel(1);                 //Escalonamento para a faixa de 1500 a 2000 us (pitch)
    receiver_input_channel_2 = convert_receiver_channel(2);                 //Escalonamento para a faixa de 1500 a 2000 us (roll)
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Escalonamento para a faixa de 1500 a 2000 us (throttle)
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Escalonamento para a faixa de 1500 a 2000 us (yaw)
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Cálculo PID Roll -------------------------------------------------------------------------------------------------------------------------
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;								// Cálculo do erro do ângulo roll
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;								// Ganho controlador I multiplicado pelo erro
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;					// Delimita esse valor para ficar entre +/- o valor máximo
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;	
// PID ROLL: ------|		Ganho_P * Erro [P]		 |Ganho_I * Erro [I]| 		Ganho_D * (Erro[i]-Erro[i-1]) [D]                 |------------
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;					// Delimita esse valor para ficar entre +/- o valor máximo
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;	// + 200 ou -200
  pid_last_roll_d_error = pid_error_temp;											// Salva o erro para o prox cálculo

  //Cálculo PID Pitch -------------------------------------------------------------------------------------------------------------------------
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;							// Cálculo do erro do ângulo roll
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;								// Ganho controlador I multiplicado pelo erro
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;				// Delimita esse valor para ficar entre +/- o valor máximo
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;// +200 ou -200
// PID PITCH: ------|		Ganho_P * Erro [P]		  |Ganho_I * Erro [I]| 		Ganho_D * (Erro[i]-Erro[i-1]) [D]          |------------
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;					// Delimita esse valor para ficar entre +/- o valor máximo
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;  // + 200 ou -200
  pid_last_pitch_d_error = pid_error_temp;                                              // Salva o erro para o prox cálculo
  
  //Cálculo PID Pitch -------------------------------------------------------------------------------------------------------------------------
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;								// Cálculo do erro do ângulo roll
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;                             	// Ganho controlador I multiplicado pelo erro
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;                   	// Delimita esse valor para ficar entre +/- o valor máximo
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
// PID PITCH: ------|		Ganho_P * Erro [P]	   |Ganho_I*Erro [I]| 		Ganho_D * (Erro[i]-Erro[i-1]) [D]          |----------------------
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;						// Delimita esse valor para ficar entre +/- o valor máximo
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;      // + 200 ou -200
  pid_last_yaw_d_error = pid_error_temp;                                            // Salva o erro para o prox cálculo
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

  actual = receiver_input[channel];                                              //Lê o valor atual do receptor (1000 a 2000 us)
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //valor BAIXO para esse canal
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //valor de CENTRO para esse canal
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //valor ALTO para esse canal

  if(actual < center){                                                         //Caso o valor atual seja menor que o de centro
    if(actual < low)actual = low;                                              //Limite mais baixo do sinal
    difference = ((long)(center - actual) * (long)500) / (center - low);       //A diferença sempre estará entre 0 e 500
    if(reverse == 1)return 1750 + difference/2;                                //Caso o canal esteja invertido
    else return 1750 - difference/2;                                           //Caso não esteja invertido
  }
  else if(actual > center){                                                    //Quando o valor recebido é maior que o de CENTRO
    if(actual > high)actual = high;                                            //Limite do valor mais baixo, adquirido no programa de configuração
    difference = ((long)(actual - center) * (long)500) / (high - center);      //A diferença sempre estará entre 0 e 500
    if(reverse == 1)return 1750 + difference/2;                                //Caso o canal esteja invertido
    else return 1750 + difference/2;                                           //Caso não esteja invertido
  }
  else return 1750;  														   //Quando é exatamente igual o de centro
}

void set_gyro_registers(){
  //Configuração da MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                      //Inicia nova cominicação I²C com o endereço configurado
    Wire.write(0x6B);                                                          //Escrita no registrador PWR_MGMT_1 endereço 0x6B hexdecimal
    Wire.write(0x00);                                                          //Ativar o giroscópio enviando 0x00
    Wire.endTransmission();                                                    //Termina a transmissão

    Wire.beginTransmission(gyro_address);                                      //Inicia nova cominicação I²C com o endereço configurado
    Wire.write(0x1B);                                                          //Escrita no registrador GYRO_CONFIG endereço 0x1B hex
    Wire.write(0x08);                                                          //Envia bits 00001000 para escala de 500dps (graus por segundo)
    Wire.endTransmission();                                                    //Termina a transmissão

    Wire.beginTransmission(gyro_address);                                      //Inicia nova cominicação I²C com o endereço configurado
    Wire.write(0x1C);                                                          //Escrita no registrador ACCEL_CONFIG endereço 0x1A hex
    Wire.write(0x10);                                                          //Insere os bits 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //Termina a transmissão

    //Checagem dos registradores
    Wire.beginTransmission(gyro_address);                                      //Inicia nova cominicação I²C com o endereço configurado
    Wire.write(0x1B);                                                          //Escrita no registrador GYRO_CONFIG endereço 0x1B hex
    Wire.endTransmission();                                                    //Termina a transmissão
    Wire.requestFrom(gyro_address, 1);                                         //Requisição de 1 byte do giroscópio
    while(Wire.available() < 1);                                               //Espera a recepção de 6 bytes
    if(Wire.read() != 0x08){                                                   //Checagem para ver se o valor é 0x08
      digitalWrite(12,HIGH);                                                   //Liga LED de aviso
      while(1)delay(10);                                                       //Fica num loop infinito
    }

    Wire.beginTransmission(gyro_address);                                      //Inicia a comunicação no endereço encontrado na configuração
    Wire.write(0x1A);                                                          //Escrita no registrador CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Insere os bits 00000011 (Filtro passa baixa ~43Hz)
    Wire.endTransmission();                                                    //Termina a transmissão    

  }  
}

