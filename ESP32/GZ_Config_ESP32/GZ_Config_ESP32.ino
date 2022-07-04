/////CALIBRAÇÃO INICIAL (EEPROM) DO DRONE CRONUS [ESP32] YOUTUBE: GRAVIDADE ZERO/////
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

#include <Wire.h>               // Biblioteca responsável pela comunicação I²C
#include <EEPROM.h>             // Biblioteca resposável por armazenar informações na memória da ESP32

#define canal_1_r 4				// GPIO 4  -> Canal 1 do receptor
#define canal_1_r 2				// GPIO 2  -> Canal 1 do receptor
#define canal_1_r 14			// GPIO 14 -> Canal 1 do receptor
#define canal_1_r 15			// GPIO 15 -> Canal 1 do receptor

gpio_config_t config_IO; 		// Struct de configuração dos GPIOs da ESP32

uint8_t Clear_buffer;


void setup() {
  //config_IO.mode = GPIO_MODE_OUTPUT;	// Pinos de saída (LED) -------------------------------------
// GPIOs 33 output [é necessário modificar o esp32 cam: https://www.youtube.com/watch?v=IUJCueFmTjk&list=PL8tB5EudLln8iGmFZsG5i9txIkICYb_jL&index=2&t=325s&ab_channel=JohnLin]  
  //config_IO.pin_bit_mask = ((uint64_t) 1 << 33);
  //gpio_config(&config_IO);            // Endereço para o struct de configuração
  config_IO.mode = GPIO_MODE_INPUT 		//Pinos de entrada (Receptor) -------------------------------
  config_IO.pin_bit_mask = ((uint64_t) 1 << 33);
  Clear_buffer = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
  GPIO.out_w1ts = 0b100; 			    //Bit set para o GPIO 2
  vTaskDelay(2000/portTICK_PERIOD_MS);	//Delay 2 seg
  GPIO.out_w1tc = Clear_buffer; 				//Bit clear GPIO para o 2 to HIGH
  vTaskDelay(2000/portTICK_PERIOD_MS);	//Delay 2 seg
}
 
