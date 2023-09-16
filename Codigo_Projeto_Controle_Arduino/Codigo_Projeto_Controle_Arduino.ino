

/* 
#############################################
# CÓDIGO PROJETO DE CONTROLE COM ARDUINO    #
# VERSÃO 3.1.0 16 DE SETEMBRO DE 2023       #
# DESENVOLVIDO POR LEANDRO FAVARETTO        #
# PARCERIA COM O PET ENGENHARIA QUIMICA UEM #
# DIFICULDADES ENTRAR EM CONTATO NO E-MAIL  #
# LEANDRO.FAVARETTO26@GMAIL.COM             #
#############################################
*/




/*INCLUSÃO DE BIBLIOTECAS NECESSÁRIAS */
#include <SoftwareSerial.h> //biblioteca necessária para se criar uma NOVA porta serial, além da porta serial USB padrão do arduino, pois o dimmer usa conexão serial...
//mas já estaremos usando a porta serial do arduino para se comunicar com o computador, portanto precisaremos usar uma porta nova para a comunicação serial
#include <Thermistor.h> //nec. para ler a temperatura dos sensores de temperatura. estamos usando as configurações padrão da bilbioteca.
//biblioteca baixada de https://drive.google.com/file/d/1pMHZLaTsk4-FDYhh27qOn9C7-MOzg1vi/view


/*DECLARAÇÃO DE PINOS*/

/*PINOS RELACIONADOS À SOFTWARE SERIAL E AO DIMMER AC*/
const int rxPin = 11;
const int txPin = 12; //porta 12 transmite informações para o Dimmer AC (é o pino RX do Dimmer, necessário fazer essa conexão elétrica)
//a porta 12 do arduino será por onde transmitiremos informação para controlar o dimmer AC (tx = transmitter)

/*PINOS RELACIONADOS AO SENSOR DE VAZÃO*/
const int sensorPin       = 2; // porta 2 será a conexão elétrica com o sensor de vazão. 
const int sensorInterrupt = 0;  // qual interruptor do Arduino será utilizado? Como estamos usando o UNO com o pino 2, é o zero. se vc estiver usando outra placa, deve alterar e ler qual interrupt é.
//interrupt 0 no Arduino Uno é o pino digital 2, pino do sensor de VAZAO (precisa ser o 2)

/*PINOS RELACIONADOS À BOMBA E AO DRIVER DA BOMBA*/
const int IN1 = 9; // saida 'positiva' do Driver da bomba (varia com PWM)
const int IN2 = 8; // saida 'negativa' do Driver da bomba (em geral, tem 1V em relação ao GND)
const int pinPump = 10; // pino que controla a velocidade de giro da bomba
// vamos enviar informação para o pino 10, indicando quantos % queremos acionar, e daí o driver vai fazer esse controle
const int pinBuzzer = 3; //pino do buzzer - buzzer nada mais é que um mini somzinho para fazer apitos
// não é necessário conectá-lo ao arduino. na verdade nunca sequer testamos se está funcionando

/* SEÇÃO DE DEFINIÇÃO DE PARÂMETROS DE CONTROLE */
/*CONTROLE DA VAZÃO*/
int controlTypePump = 0; //0 - MANUAL 1 - ON/OFF 2 - Proporcional 3 - Proporcional-Integral 4 - Proporcional-Integral-Derivativo 5 - PIDf
float flowSetpoint = 35; //setpoint da vazão L/hr (faixa de operação da bomba 0 - 60 L/hr)
float kcPump = 2; //ganho do controlador da bomba, dado nas unidades de %hr/L.
//nos relatórios da IC, os valores tão multiplicados por 60, pois a medida antes era em L/min.
float tauIPump = 2; //tau I em segundos. lembrando das aulas de controle, se tauI tende a infinito, o termo integral tende à zero.
//portanto, ao iniciarmos o controle com um tauI grande, estamos iniciando com o controle integral desativado.
float tauDPump = 1; //tau D em segundos. iniciando em zero, iniciamos com o controle derivativo desativado.
float i=0; //variavel auxiliar que será usada para armazenar o valor da porcentagem de acionamento da bomba.
float oldErrorPump = 0; //essa é uma variável global que vai armazenar o erro anterior de controle, ou seja, o erro na última medida (equivalente à e_k-1)
float oldTimeControlPump = 0; //essa é uma variável global que vai armazenar o último tempo, ou seja, o tempo da última medida (EM milissegundos)
float oldIntegralPump = 0; //essa é uma variável global que vai armazenar o último valor da integral da bomba, para ser usado no controle integral.
float hysteresisFlow = 10; //valor da histerese inicial.
float biasPump = 0; //bias para ser usado no controle proporcional

float pumpPower = 0; //variavel auxiliar que será usada para armazenar o valor da porcentagem de acionamento da potencia da bomba (NOVA)
float oldFlow = 0; //variavel que vai armazenar o valor da variavel controlada de 1 instante de temp atrás
float oldOldFlow = 0; //variavel que vai armazenar o valor da variavel controlada de 2 instantes de temp atrás
float oldDerivativeActionPump = 0; //variavel armazena o valor da ultima ação derivativa da bomba
float upperLimitPump = 100; //limite superior da bomba
float lowerLimitPump = 0; //limite inferior da bomba

/*CONTROLE DA TEMPERATURA*/
float tempSetpoint = 30; //setpoint da temperatura em graus celsius
int controlTypeRes = 0; //0 - MANUAL 1 - ON/OFF 2 - Proporcional 3 - Proporcional-Integral 4 - Proporcional-Integral-Derivativo 5 - PIDf 6 - CASCATAP 7 - CASCATAPI 8 - CASCATAPID 9 - FEEDFORWARD+P 10 - FFPI 11 - FFPID tipo de controle da RESISTENCIA
float kcRes = 500; //ganho do controlador da resistência, dado nas unidades de 1/°C
float tauIRes = 10; //tau I em segundos. lembrando das aulas de controle, se tauI tende a infinito, o termo integral tende à zero.
float tauDRes = 5; //tau D em segundos. iniciando em zero, iniciamos com o controle derivativo desativado.
float j=0; //variavel auxiliar que será usada para armazenar o valor da porcentagem de acionamento da resistencia.
float oldErrorRes = 0; //essa é uma variável global que vai armazenar o erro anterior de controle, ou seja, o erro na última medida (equivalente à e_k-1)
float oldTimeControlRes = 0; //essa é uma variável global que vai armazenar o último tempo, ou seja, o tempo da última medida (EM milissegundos)
float oldIntegralRes = 0; //essa é uma variável global que vai armazenar o último valor da integral da resistencia, para ser usado no controle integral.
float hysteresisTemp = 3; //valor da histerese inicial.
float biasRes = 0; //bias para ser usado no controle proporcional-integral

float resPower = 0; //variavel auxiliar que será usada para armazenar o valor da porcentagem de acionamento da potencia da resistencia (NOVA)
float oldTemp = 0; //variavel que vai armazenar o valor da variavel controlada de 1 instante de temp atrás
float oldOldTemp = 0; //variavel que vai armazenar o valor da variavel controlada de 2 instantes de temp atrás
float oldDerivativeActionRes = 0; //variavel armazena o valor da ultima ação derivativa da resistencia
float upperLimitRes = 100; //limite superior da resistência
float lowerLimitRes = 0; //limite inferior da resistência


/* CONTROLE AVANÇADO TEMPERATURA - CASCATA E FEEDFORWARD */
//Está aqui, mas continua a implementação na forma de POSIÇÃO. Tem que ser implementada a forma de VELOCIDADE/INCREMENTAL para que se possa tirar o biasFlowSetpoint.
//Tem que ler o capítulo 14, 15 e 16 lá e buscar no Solution do Seborg 4a edição pra buscar a forma do controle PID + feedforward digital na forma de velocidade.
float biasFlowSetpoint = 0;
float oldj = 0;
float oldFlowAvg = 0;
float kFF = 0;
float tau1FF = 100;
float tau2FF = 100;

/* SEÇÃO DE CONFIGURAÇÕES DOS SENSORES */
float interval = 250; //intervalo de tempo entre leituras do sensor de vazao e também do controle em milissegundos
//ver início do capítulo 17 do livro Process Seborg 4a edição, equação 17-3a e 17-3b
float calibrationFactor = 78.5; //Fator de calibração. Quantos Hertz equivalem a 1 L/min? Use o FreqCount, código pronto para leitura de frequência e calibre vc mesmo.
volatile byte pulseCount = 0; // variavel auxiliar na contagem de pulsos
unsigned long oldTimeFlow = 0; // tempo inicial do arduino, tempo de quanto tempo passou desde a finalização da amostragem de tempo.
unsigned long oldTimeSampling = 0; //tempo inicial do arduino, tempo anterior de amostragem de tempo.
float totalMilliLitres = 0; // volume inicial
float flowMax = 72; //se a vazao passar disso ele trava a leitura nesse valor para não atrapalhar o controle. é sabido de problemas no leitor do sensor de temperatura
//int flowSpikeProtectionMax = 75; //quantidade de vezes que ele tolera a vazão ficar maluca seguido.
int flowSpikeCount = 0;

/* SEÇÃO DE CONFIGURAÇÕES DAS BIBLIOTECAS UTILIZADAS */
SoftwareSerial newSerial =  SoftwareSerial(rxPin, txPin); //criando a nova porta serial usando os pinos de transmissão e recepção de informação definidos anteriormente
Thermistor sensorTIN(0); //VARIÁVEL DO TIPO THERMISTOR, INDICANDO O PINO ANALÓGICO (A0) EM QUE O TERMISTOR ESTÁ CONECTADO
Thermistor sensorTOUT(4); //VARIÁVEL DO TIPO THERMISTOR, INDICANDO O PINO ANALÓGICO (A4) EM QUE O TERMISTOR ESTÁ CONECTADO

/* CONFIGURAÇÕES DAS MÉDIAS UTILIZADAS */
/* MÉDIA MÓVEL */
int mediaMovelT = 10; //qual é o tamanho da média movel utilizada. 1 para desligado. N ultimos pontos, max = 40.
int mediaMovelFlow = 8; //qual é o tamanho da média movel utilizada, 1 para desligado. N ultimos pontos, max = 40.

//vetor que irá armazenar os antigos dados da média móvel. 
float tempInVector[40];
int indexIn = 0;
float tempOutVector[40];
int indexOut = 0;
float flowVector[40];
int indexFlow = 0;

/*MÉDIA MOVEL EXPONENCIAL*/
//para desligar a média movel exponencial, fazer alfa = 1.
float alfaTemp = 0.1; //recomenda-se valores entre 0.05 e 0.3. 0.1 indica que a medida atual tem o valor somente de 10%, sendo o 90% sendo o peso das medidas anteriores.
float alfaFlow = 0.4; //recomenda-se valores entre 0.3 e 0.8. 0.1 indica que a medida atual tem o valor somente de 10%, sendo o 90% sendo o peso das medidas anteriores.
float oldTempIn; //valor antigo da temperatura de entrada.
float oldTempOut; //valor antigo da temperatura de saida.
float alfaDerivativePump = 0.2; //média móvel exponencial da saída derivativa da bomba
float alfaDerivativeRes = 0.2; //média móvel exponencial da saída derivativa da resistencia

/*INPUT PELA SERIAL*/
char command[20];
int commandIndex = 0;

// Variáveis globais
bool rampFlowActive = false;   // Indica se a rampa está em progresso
bool rampTempActive = false;
float rampInitialTime = 0;
float rampInitialPoint = 0;
float rampFinalTime = 0.0;  // Duração da rampa em segundos
float rampTarget = 0.0;    // Valor final da rampa


/*SEÇÃO DO CÓDIGO DO ARDUINO*/
void setup()
{
  Serial.begin(115200); //inicializa porta serial com velocidade de transmissão de 115200 (ao usar o monitor serial use essa velocidade)
  newSerial.begin(9600); //inicializa a nossa software serial (porta 12) com velocidade de transmissão de 9600 também (o dimmer PRECISA usar essa velocidade)
  pinMode(sensorPin, INPUT); //inicialização de pino do sensor de vazão como input
  digitalWrite(sensorPin, HIGH); //inicialização do pino do sensor como LIGADO (os pulsos são contados na queda, por isso tem que começar ligado)
  
  pinMode(IN1,OUTPUT); //inicializa porta 'positiva' do driver como saida
  pinMode(IN2,OUTPUT);//inicializa porta 'negativa' do driver como saida
  pinMode(pinPump,OUTPUT); //inicializa porta de controle da velocidade da bomba como saida
  digitalWrite(IN1,HIGH); // esses dois comandos definem a direção de giro da bomba. se fosse invertido inverteriamos os estados
  digitalWrite(IN2,LOW);
  pinMode(pinBuzzer,OUTPUT); //inicializa porta do buzzer como saída. lembrando que precisa ser uma porta de PWM.
  manipulatePump(pumpPower); //inicializa a bomba com a velocidade pumpPower
  manipulateRes(resPower);
  /*The Hall-effect sensor is connected to pin 2 which uses interrupt 0. Configured to trigger on a FALLING state change (transition from HIGH
  (state to LOW state)
  Aqui estamos configurando como irá funcionar a interrupção do nosso arduino. Toda vez que houver uma QUEDA (FALLING) de tensão no 
  pino do sensor de vazão, o procedimento "pulseCounter" irá rodar, e enquanto o procedimento estiver rodando, o tempo de processamento
  do millis() não será contado.
  Em suma, configuramos como será feita a contagem de pulsos. Posteriormente será usado para medição de frequencia.*/
  //attachInterrupt(sensorInterrupt, pulseCounter, FALLING); //you can use Rising or Falling
  
}

void loop()
{
/* SEÇÃO DE RECEBIMENTO DE INFORMAÇÕES DA PORTA SERIAL */
while (Serial.available() > 0) {
  char c = Serial.read();

  if (c == '\n') { // Fim do comando
    command[commandIndex] = '\0'; // Adiciona o caractere nulo ao final da string
    processCommand(command);
    commandIndex = 0; // Reiniciar o índice para a próxima linha
  } else if (commandIndex < sizeof(command) - 1) {
    command[commandIndex] = c;
    commandIndex++;
  }
}

/*SEÇÃO DE RAMPAS DO SETPOINT, SE ESTIVEREM ATIVAS*/
if (rampFlowActive == true) 
{    
    // Verificar se a rampa já atingiu a duração especificada
    if (float(millis())/1000 >= rampFinalTime)
    {
      // A rampa atingiu a duração, definir o setpoint diretamente para o valor alvo
      flowSetpoint = rampTarget;
      buzzerOK();
      
      // Desativar a rampa
      rampFlowActive = false;
    } 
    else 
    {
      // A rampa ainda está em progresso, calcular o setpoint incremental
      flowSetpoint = rampInitialPoint + (float(millis())/1000 - rampInitialTime)*(rampTarget - rampInitialPoint)/(rampFinalTime - rampInitialTime);
      buzzerOK();
    }
}

if (rampTempActive == true) 
{    
    // Verificar se a rampa já atingiu a duração especificada
    if (float(millis())/1000 >= rampFinalTime)
    {
      // A rampa atingiu a duração, definir o setpoint diretamente para o valor alvo
      tempSetpoint = rampTarget;
      buzzerOK();
      
      // Desativar a rampa
      rampTempActive = false;
    } 
    else 
    {
      // A rampa ainda está em progresso, calcular o setpoint incremental
      tempSetpoint = rampInitialPoint + (float(millis())/1000 - rampInitialTime)*(rampTarget - rampInitialPoint)/(rampFinalTime - rampInitialTime);
      buzzerOK();
    }
}





/*SEÇÃO DE CÓDIGOS QUE É REALIZADA A INTERVALO DE TEMPO DE AMOSTRAGEM*/

  if((millis() - oldTimeSampling) > interval)    //Fazer esse cálculo somente quando o intervalo de amostragem passar.
   // ou seja, o if só é valor quando já se passarem interval milissegundos desde o último instante.
  { 
    oldTimeSampling = millis();
    /* SEÇÃO DE TOMADA DA VAZÃO */

    // Dsabilitar a interrupção enquanto estivermos calculando a vazão da bomba.
    // Ou seja, enquanto está calculando a vazão da bomba, não vai mais contar os pulsos. Depois vai voltar a contar.
    detachInterrupt(sensorInterrupt);

    // Because this loop may not complete in exactly 1 second intervals we calculate the number of milliseconds that have passed since the last execution and use that to scale the output. We also apply the calibrationFactor to scale the output based on the number of pulses per second per units of measure (litres/minute in this case) coming from the sensor.
    // Esse loop pode não completar em um segundo, por isso, calculamos o numero de milissegundos que passaram desde a última execução e fazemos essa correção. (Não completa em um segundo pq o arduino tem que processar o que está dentro do if).
    //Aqui é feita uma extrapolação. Calcula-se a frequência no intervalo mencionado e extrapola-se 1 segundo, para obtermos a vazão total.
    // Multiplicado por 60 para em L/hr
    float flowRate = 60*((interval / (millis() - oldTimeFlow)) * pulseCount)*1000 / (interval*calibrationFactor);

 
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    float flowMilliLitres = (1000*flowRate / 3600); //converte vazão de L/hr para mL/hr e depois divide por 60 para saber quantos mL estão passando.

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres; //é literalmente uma integração

    /* SEÇÃO DE TOMADA DAS TEMPERATURAS */
    float tempIn = sensorTIN.getTemp();
    float tempOut = sensorTOUT.getTemp();

   

    /*SEÇÃO DE CRIAÇÃO DO VETOR DE MEDIDAS PARA CÁLCULO DA MÉDIA MÓVEL */
    //assignação da temperatura de entrada na última casa do vetor
     if (isnan(tempIn) == false)
    {

      tempInVector[indexIn] = tempIn;

      //se o índice já chegou até o final, reinicia, armazena na casa zero.
      //se não, aumenta um no índice.
      if (indexIn == mediaMovelT - 1)
      {
        indexIn = 0;
      }
      else
      {
        indexIn++;
      }

    }
    else
    {
      tempIn = 0;
    }


    if (isnan(tempOut) == false)
    {

      tempOutVector[indexOut] = tempOut;

      //se o índice já chegou até o final, reinicia, armazena na casa zero.
      //se não, aumenta um no índice.
      if (indexOut == mediaMovelT - 1)
      {
        indexOut = 0;
      }
      else
      {
        indexOut++;
      }

    }
    else
    {
      tempOut = 0;
    }
    //mesma coisa agora a vazão.

    /*FILTRO SPIKE VAZÃO */
    //Filtro contra spike de vazão. É sabido que a bomba não bombeia a uma vazão superior à flowMax, portanto, quando medidas superiores à essa aparecem, devemos desconsiderá-las e repetir a medida anterior.
    // Após um certo tempo ele desliga a bomba. Quando isso acontecer, usar somente <80% < 40 L/h
    //TO-DO: implementar um check melhor. Esse é muito rudimentar. Avaliar a média movel da vazão, desvio-padrão, algo assim.
    if (flowRate > flowMax) 
    {
      //flowRate = oldFlowAvg;
      //flowSpikeCount++;
      //if (flowSpikeCount > flowSpikeProtectionMax)
      //{
      //      controlTypePump = 0;
      //      oldFlow = 0;
      //      oldOldFlow = 0;
      //      oldErrorPump = 0;
      //      i = 0;
      //      buzzerOK();
      //      pumpPower = 0;
      //      flowSpikeCount = 0;
      //}
    }

    flowVector[indexFlow] = flowRate;

    if (indexFlow == mediaMovelFlow - 1)
    {
      indexFlow = 0;
    }
    else
    {
      indexFlow++;
    }

    /* SEÇÃO CÁLCULO DAS MÉDIAS MÓVEIS*/
    float flowAvg = average(flowVector,mediaMovelFlow);
    float tempInAvg = average(tempInVector,mediaMovelT);
    float tempOutAvg = average(tempOutVector,mediaMovelT);
    
    /* SEÇÃO CÁLCULO DAS MÉDIAS MÓVEIS EXPONENCIAIS */
    //CALCULAR A MÉDIA MÓVEL SOMENTE APÓS O VETOR DE MEDIDAS ESTAR COMPLETAMENTE PREENCHIDO. 
    //ESSE TEMPO DE PREENCHIMENTO DO VETOR DEPENDE DO TAMANHO DELE E DO INTERVALO DE MEDIDAS.
    if (float(millis())<(max(mediaMovelT,mediaMovelFlow)-1)*interval)
    {
      //Se essa condição é verdadeira, o vetor de medidas ainda não foi completamete preenchido, e há algum elemento que é igual a zero (ou NaN)
      //portanto, há um Bypass no calculo da média movel e também da exponencial.
      oldTempIn = tempIn;
      oldTempOut = tempOut;
      oldFlowAvg = flowRate;
    }
    else
    { 
      //o vetor já está completamente preenchido e portanto você pode calcular a média movel normal e também a média móvel exponencial.
      //lembrando que o código está considerando essas duas médias em SÉRIE. ou seja, como dois filtros acoplados em seguida.

      if (tempIn == 0)
      {
        tempInAvg = 0;
      }
      else
      {
        tempInAvg = tempInAvg*alfaTemp + (1-alfaTemp)*oldTempIn;
        oldTempIn = tempInAvg;
      }

      if (tempOut == 0)
      {
        tempOutAvg = 0;
      }
      else
      {
        tempOutAvg = tempOutAvg*alfaTemp + (1-alfaTemp)*oldTempOut;
        oldTempOut = tempOutAvg;
      }

      flowAvg = flowAvg*alfaFlow + (1-alfaFlow)*oldFlowAvg;
      oldFlowAvg = flowAvg;

    }
    
    switch (controlTypePump)
    {
      case 0: //MANUAL
        //essa linha aqui é sem checar saturação.
        i = checkSaturation(i,pumpPower-i,lowerLimitPump,upperLimitPump);
        biasPump = i;
        break;
      case 1: //ON-OFF
        i = onoff(flowAvg,flowSetpoint,hysteresisFlow,i,lowerLimitPump,upperLimitPump);
        biasPump = i;
        break;
      case 2: //P
        //o bias usado é o valor da última potência da bomba acionada. admite-se que está em Regime Permanente.
        biasPump = 30 + flowSetpoint*1.054; //relação empirica baseada no ganho
        i = controlPPosition(kcPump,flowAvg,flowSetpoint,biasPump,lowerLimitPump,upperLimitPump);
        break;
      case 3: //PI
        i = controlPIPumpVelocity(kcPump,tauIPump,flowAvg,flowSetpoint,i,lowerLimitPump,upperLimitPump);
        biasPump = i;
        break;
      case 4: //PID
        i = controlPIDPumpVelocity(kcPump,tauIPump,tauDPump,flowAvg,flowSetpoint,i,lowerLimitPump,upperLimitPump);
        biasPump = i;
        break;
      case 5:
        i = controlPIDFPumpVelocity(kcPump,tauIPump,tauDPump,flowAvg,flowSetpoint,i,lowerLimitPump,upperLimitPump);
        biasPump = i;
        break;
    }
    oldTimeControlPump = millis();
    manipulatePump(i); //aciona a bomba com a potência calculada

    switch (controlTypeRes)
    {
      case 0: //MANUAL
        //j = resPower; //essa linha aqui é sem checar saturação.
        j = checkSaturation(j,resPower-j,lowerLimitRes,upperLimitRes);
        biasRes = j;
        biasFlowSetpoint = flowSetpoint;
        break;
      case 1: //ON-OFF
        j = onoff(tempOutAvg,tempSetpoint,hysteresisTemp,j,lowerLimitRes,upperLimitRes);
        biasRes = j;
        biasFlowSetpoint = flowSetpoint;
        break;
      case 2: //P
        //o bias usado é o valor da última potência da bomba acionada. admite-se que está em Regime Permanente.
        j = controlPPosition(kcRes,tempOutAvg,tempSetpoint,biasRes,lowerLimitRes,upperLimitRes);
        biasFlowSetpoint = flowSetpoint;
        break;
      case 3: //PI
        j = controlPIResVelocity(kcRes,tauIRes,tempOutAvg,tempSetpoint,j,lowerLimitRes,upperLimitRes);
        biasRes = j;
        biasFlowSetpoint = flowSetpoint;
        break;
      case 4: //PID
        j = controlPIDResVelocity(kcRes,tauIRes,tauDRes,tempOutAvg,tempSetpoint,j,lowerLimitRes,upperLimitRes);
        biasRes = j;
        biasFlowSetpoint = flowSetpoint;
        break;
      case 5: //PIDF
        j = controlPIDFResVelocity(kcRes,tauIRes,tauDRes,tempOutAvg,tempSetpoint,j,lowerLimitRes,upperLimitRes);
        biasRes = j;
        biasFlowSetpoint = flowSetpoint;
        break;
      case 6: //Cascata P
        j = resPower;
        flowSetpoint = controlPPosition(kcRes,tempOutAvg,tempSetpoint,biasFlowSetpoint,0,70);
        break;
      case 7: //Cascata PI
        j = resPower;
        flowSetpoint = controlPIResVelocity(kcRes,tauIRes,tempOutAvg,tempSetpoint,j,0,70);
        biasFlowSetpoint = flowSetpoint;
        break;
      case 8: //Cascata PID
        j = resPower;
        flowSetpoint = controlPIDResVelocity(kcRes,tauIRes,tauDRes,tempOutAvg,tempSetpoint,j,0,70);
        biasFlowSetpoint = flowSetpoint;
        break;
      //case 9:
        //oldj = j;
        //j = feedForwardPPosition(kcRes,tempOutAvg,tempSetpoint,kFF,tau1FF,tau2FF,flowAvg,resPower,lowerLimitRes,upperLimitRes);
        //break;
      //case 10:
        //oldj = j;
        //j = feedForwardPIPosition(kcRes,tauIRes,tempOutAvg,tempSetpoint,kFF,tau1FF,tau2FF,flowAvg,resPower,lowerLimitRes,upperLimitRes);
        //break;
      //case 11:
        //oldj = j;
        //j = feedForwardPIPosition(kcRes,tauIRes,tempOutAvg,tempSetpoint,kFF,tau1FF,tau2FF,flowAvg,resPower,lowerLimitRes,upperLimitRes);
        //break;
    }
    oldTimeControlRes = millis();
    manipulateRes(j); //aciona a bomba com a potência calculada


    /*SEÇÃO DE PRINT DE DADOS PARA COMUNICAÇÃO COM A INTERFACE */
    //Hoje, temos a seguinte estrutura de comunicação, em ordem, separado por somente UM espaço.
    //tempo_desde_inicio_em_ms vazao_filtrada setpoint_vazao potencia_bomba histerese_vazao temp_in_filtrada temp_out_filtrada setpoint_temp potencia_res histerese_temp vazao_antesfiltro temp_saida_antesfiltro temp_entrada_antesfiltro 
    Serial.print(float(float(millis())/1000),2);

    Serial.print(" ");
    
    Serial.print(flowAvg, 2);  // Print the integer part of the variable      

    Serial.print(" ");  
    
    Serial.print(flowSetpoint);
    
    Serial.print(" ");

    Serial.print(float(int(i*255/100 + 0.5))*100/255);
    
    Serial.print("% ");  

    Serial.print(hysteresisFlow);
    
    Serial.print(" ");

    Serial.print(tempInAvg,1);
    
    Serial.print(" ");
    
    Serial.print(tempOutAvg,1);
    
    Serial.print(" ");
    
    Serial.print(tempSetpoint,1);
    
    Serial.print(" ");

    Serial.print(float(int(j*22/100 + 0.5))*100/22);
    
    Serial.print("% ");

    Serial.print(hysteresisTemp,1);

    Serial.print(" ");

    Serial.print(flowRate);

    Serial.print(" ");

    Serial.print(tempOut);

    Serial.print(" ");

    Serial.println(tempIn);

    // Reset the pulse coun1ter so we can start incrementing again
    //Zera todos os pulsos contados até agora.
    pulseCount = 0;
    
      // Habilitar o interrupt novamente agora que já finalizamos todos os cálculos, retomada da contagem de pulsos!
    // A água que passou pelo sensor enquanto estávamos fazendo os cálculos não foi contada.
    // Retomar a contar água!
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

     // Após finalização de todo o código, retomar o contador de tempo.
    oldTimeFlow = millis();
  }

}

//controle por histerese
float onoff(float y, float ysp, float hysteresis, float atual, const int lowerlimit, const int upperlimit)
{
  float u;
  if (y > ysp + hysteresis) {u = lowerlimit;}
  else
    {
    if (y < ysp - hysteresis) 
      {u = upperlimit;}
      else
      {
       u = atual; 
      }
    }
  return u;
}

//controle proporcional genérico na forma de posição.
float controlPPosition(float kc,float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float u = P;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  return u;
}

// controle proporcional da vazão modo velocidade
float controlPPumpVelocity(float kc, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  float error = ysp - y;
  float deltaPower = kc * (error - oldErrorPump);
  oldErrorPump = error;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}

// controle proporcional integral da vazão modo velocidade
float controlPIPumpVelocity(float kc, float tauI, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  float error = ysp - y;
  float deltaPower = kc * ( (error - oldErrorPump) + (interval/1000)*error/tauI );
  oldErrorPump = error;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}

// controle proporcional integral derivativo da vazão modo velocidade
float controlPIDPumpVelocity(float kc, float tauI, float tauD, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  
  //estudar por um filtro de primeira ordem no termo derivativo.
  float error = ysp - y;  
  float deltaPower = kc * ( (error - oldErrorPump) + (interval/1000)*error/tauI + -tauD*(y - 2*oldFlow + oldOldFlow)/(interval/1000) );
  oldErrorPump = error;
  oldOldFlow = oldFlow;
  oldFlow = y;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}

// controle proporcional integral derivativo com filtro da vazão modo velocidade
float controlPIDFPumpVelocity(float kc, float tauI, float tauD, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  
  float error = ysp - y;  
  float derivativeAction = -tauD*(y - 2*oldFlow + oldOldFlow)/(interval/1000)*alfaDerivativePump + (1-alfaDerivativePump)*oldDerivativeActionPump;
  float deltaPower = kc * ( (error - oldErrorPump) + (interval/1000)*error/tauI + derivativeAction );
  oldErrorPump = error;
  oldOldFlow = oldFlow;
  oldFlow = y;
  oldDerivativeActionPump = derivativeAction;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}



// controle proporcional da temperatura modo velocidade
float controlPResVelocity(float kc, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  float error = ysp - y;
  float deltaPower = kc * (error - oldErrorRes);
  oldErrorRes = error;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}

// controle proporcional integral da temp modo velocidade
float controlPIResVelocity(float kc, float tauI, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  
  //estudar por um filtro de primeira ordem no termo derivativo.
  float error = ysp - y;  
  float deltaPower = kc * ( (error - oldErrorRes) + (interval/1000)*error/tauI);
  oldTemp = y;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}

// controle proporcional integral derivativo da temp modo velocidade
float controlPIDResVelocity(float kc, float tauI, float tauD, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  
  //estudar por um filtro de primeira ordem no termo derivativo.
  float error = ysp - y;  
  float deltaPower = kc * ( (error - oldErrorRes) + (interval/1000)*error/tauI - tauD*(y - 2*oldTemp + oldOldTemp)/(interval/1000) );
  oldErrorRes = error;
  oldOldTemp = oldTemp;
  oldTemp = y;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}

// controle proporcional integral derivativo da vazão modo velocidade
float controlPIDFResVelocity(float kc, float tauI, float tauD, float y, float ysp, float oldPower, const int lowerlimit, const int upperlimit)
{
  
  float error = ysp - y;  
  float derivativeAction = -tauD*(y - 2*oldTemp + oldOldTemp)/(interval/1000)*alfaDerivativeRes + (1-alfaDerivativeRes)*oldDerivativeActionRes;
  float deltaPower = kc * ( (error - oldErrorRes) + (interval/1000)*error/tauI + derivativeAction );
  oldErrorRes = error;
  oldOldTemp = oldTemp;
  oldTemp = y;
  oldDerivativeActionRes = derivativeAction;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
}
// função que checa se haverá saturação do atuador. por exemplo potencia máxima é 100. se a potencia atual é 80, e a variação de potencia calculada foi de
// 30, a próxima potencia será de 110, mas isso é superior à 100%. essa função não permite que os limites do atuador sejam violados
// (checa se houve saturação do atuador)
// se vc por 110% na potencia da bomba, ela irá acionar 10% (pois ele passa a ciclar) por isso essa função é necessária. mesma coisa resistencia
float checkSaturation (float power, float deltaPower, const int lowerlimit, const int upperlimit)
{
  if (power + deltaPower > upperlimit)
  {
    return(upperlimit);
  }
  else
  {
    if (power + deltaPower < lowerlimit)
    {
      return(lowerlimit);
    }
    else
    {
      return (power + deltaPower);
    }
  }
}

//função usada para manipular a potencia da bomba.
void manipulatePump(float i)
{
 analogWrite(pinPump,i*255/100);
}

//função usada para manipular a potencia da res.
void manipulateRes(float i)
{
 enviaDimmer(int(i*22/100 + 0.5));
}

//Insterrupt Service Routine - O CÓDIGO QUE SERÁ EXECUTADO TODA VEZ QUE O PULSO CAIR
//Interrupt Service Routine - O CÓDIGO QUE SERÁ EXECUTADO TODA VEZ QUE O PULSO DE QUEDA FOR DETECTADO

void pulseCounter()
{
  // Increment the pulse counter
  // amenta o contador de pulsos.
  pulseCount++;
}

//isso aqui é pra tocar o jingle do PET toda vez que receber um comando. mas nunca consegui fazer funcionar.
void buzzerOK()
{
  //F5
  //E5 A4 B4 C5
  tone(pinBuzzer,698,200);
}

//essa função é usada para calcular a média de um vetor de n posições. 
//é usada no cálculo da média móvel.
float average(float vetor[], int n)
{
  float total = 0;
  int i;
      for(i=0; i<n; i++)
      {
        total=total+vetor[i];//loop for calculatin total
      }
    float avg=total/i;//calculate average
    return avg;
}

//usada para manipular a potencia da resistencia.
//essa função foi fornecida pelo próprio construtor do Dimmer - compramos do Mercado Livre.
void enviaDimmer(uint8_t nivel) //0 até 23
{
  newSerial.write(2);
  newSerial.write(1);
  newSerial.write(nivel);
  newSerial.write(3);
}

// essa função é usada para converter a cadeia de caracteres recebida pela porta serial em COMANDOS
// cada comando significa uma coisa.
void processCommand(const char* cmd) 
{
  if (strncmp(cmd, "IS ", 3) == 0) 
  {
    interval = atoi(cmd + 3);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "SPF ", 4) == 0) 
  {
    flowSetpoint = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "SPT ", 4) == 0) 
  {
    tempSetpoint = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "CTP ", 4) == 0) 
  {
    controlTypePump = atoi(cmd + 4);
    oldFlow = 0;
    oldOldFlow = 0;
    oldErrorPump = 0;
    i = 0;
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "CTR ", 4) == 0) 
  {
    controlTypeRes = atoi(cmd + 4);
    oldTemp = 0;
    oldOldTemp = 0;
    oldErrorRes = 0;
    j = 0;
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "PP ", 3) == 0) 
  {
    pumpPower = atof(cmd + 3);
    pumpPower = int(pumpPower * 255 / 100 + 0.5) * 100 / 255;
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "RP ", 3) == 0) 
  {
    resPower = atof(cmd + 3);
    resPower = int(resPower * 22 / 100 + 0.5) * 100 / 22;
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "KCP ", 4) == 0) 
  {
    kcPump = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "TIP ", 4) == 0) 
  {
    tauIPump = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "TDP ", 4) == 0) 
  {
    tauDPump = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "KCR ", 4) == 0) 
  {
    kcRes = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "TIR ", 4) == 0) 
  {
    tauIRes = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "TDR ", 4) == 0) 
  {
    tauDRes = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "HF ", 3) == 0) 
  {
    hysteresisFlow = atof(cmd + 3);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "HT ", 3) == 0) 
  {
    hysteresisTemp = atof(cmd + 3);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "KFF ", 4) == 0) 
  {
    kFF = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "T1F ", 4) == 0) 
  {
    tau1FF = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "T2F ", 4) == 0) 
  {
    tau2FF = atof(cmd + 4);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "MMT ", 4) == 0) 
  {
    mediaMovelT = atoi(cmd + 4);
    indexIn = 0;
    indexOut = 0;
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "MMF ", 4) == 0) 
  {
    mediaMovelFlow = atoi(cmd + 4);
    indexFlow = 0;
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "AT ", 3) == 0) 
  {
    alfaTemp = atof(cmd + 3);
    buzzerOK();
    return;
  }
  
  if (strncmp(cmd, "AF ", 3) == 0) 
  {
    alfaFlow = atof(cmd + 3);
    buzzerOK();
    return;
  }

  if (strncmp(cmd, "RSPF ", 5) == 0) 
  {
    float target = atof(cmd + 5);
    float duration = atof(strchr(cmd + 5, ' ') + 1);
    rampInitialTime = float(millis())/1000; //segundos!
    rampInitialPoint = flowSetpoint;

    rampTarget = target;
    rampFinalTime = duration + rampInitialTime;

    rampFlowActive = true;
    buzzerOK();
  } 

  if (strncmp(cmd, "RSPT ", 5) == 0) 
  {
    float target = atof(cmd + 5);
    float duration = atof(strchr(cmd + 5, ' ') + 1);
    rampInitialTime = float(millis())/1000; //segundos!
    rampInitialPoint = flowSetpoint;

    rampTarget = target;
    rampFinalTime = duration + rampInitialTime;
    
    rampTempActive = true;
    buzzerOK();
  } 

  if (strncmp(cmd, "ULP ", 4) == 0) 
  {
    upperLimitPump = atof(cmd + 4);
    buzzerOK();
  } 

  if (strncmp(cmd, "LLP ", 4) == 0) 
  {
    lowerLimitPump = atof(cmd + 4);
    buzzerOK();
  } 

  if (strncmp(cmd, "ULR ", 4) == 0) 
  {
    upperLimitRes = atof(cmd + 4);
    buzzerOK();
  } 

  if (strncmp(cmd, "LLR ", 4) == 0) 
  {
    lowerLimitRes = atof(cmd + 4);
    buzzerOK();
  } 

 if (strncmp(cmd, "ADP ", 4) == 0) 
  {
    alfaDerivativePump = atof(cmd + 4);
    buzzerOK();
  }

  if (strncmp(cmd, "ADR ", 4) == 0) 
  {
    alfaDerivativeRes = atof(cmd + 4);
    buzzerOK();
  }  

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// FUNÇÕES NÃO UTILIZADAS - AQUI PARA FINS DIDÁTICOS ////
// O QUE ESTÁ ABAIXO DISSO NÃO ESTÁ SENDO USADO NO CÓDIGO///
///ATENÇÃO ------- DESUSO --------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* SEÇÃO DE CÓDIGOS DE CONTROLE PI E PID NA FORMA DE POSIÇÃO - NÃO É USADA MAIS DEVIDO A WINDUP E TER QUE FORNECER O VIÉS (BIAS) */

float controlPIPumpPosition(float kc,float tauI, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em porcentagem que controla quanto de potência irá para o atuador
  float I = oldIntegralPump + kc/tauI * (error + oldErrorPump)/2 * interval/1000;
  if (I>upperlimit){I = upperlimit;}else{if(I<lowerlimit){I=lowerlimit;}}
  oldErrorPump = error;
  
  float u = P + I;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit){u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralPump = I;}
  return u;
}

float controlPIDPumpPosition(float kc,float tauI, float tauD, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  
  //estudar por um filtro de primeira ordem no termo derivativo.
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralPump + kc/tauI * (error + oldErrorPump)/2 * interval/1000;
  float D = -kc * tauD * (y - oldFlow) / (interval/1000);
  oldErrorPump = error;
  oldFlow = y;
  float u = P + I + D;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralPump = I;}
  return u;
}

float controlPIResPosition(float kc,float tauI, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (interval)/1000;
  oldErrorRes = error;
  float u = P + I;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralRes = I;}
  return u;
}

float controlPIDResPosition(float kc,float tauI, float tauD, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  //estudar por um filtro de primeira ordem no termo derivativo.
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (interval)/1000;

  float D = kc * tauD * (error - oldErrorPump) / (interval/1000);
  oldErrorRes = error;
  float u = P + I + D;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralRes = I;}
  return u;
}

/* 
float feedForwardPIPumpVelocity(float kc,float tauI,float y,float ysp,float kFF,float tau1FF, float tau2FF, float oldPower,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float error = ysp - y;
  float deltaPowerFF = -a1*oldPower +b1*error + b2*oldErrorPump - oldPower;
  float deltaPower = kc * ( (error - oldErrorPump) + (interval/1000)*error/tauI ) + deltaPowerFF;
  oldErrorPump = error;
  return(checkSaturation(oldPower,deltaPower,lowerlimit,upperlimit));
} 
*/

/* SEÇÃO DE CÓDIGOS DE CONTROLE P, PI E PID FEEDFORWARD NA FORMA DE POSIÇÃO - NÃO ESTÁ SENDO USADO DEVIDO A TER QUE FORNECER O BIAS */
float feedForwardPPosition(float kc,float y,float ysp,float kFF,float tau1FF, float tau2FF, float y1,float Pbias,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float u = -a1*oldj + b1*y1 + b2*oldFlowAvg;

  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  u = u + P;
  
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  
  return u;
}

float feedForwardPIPosition(float kc,float tauI,float y,float ysp,float kFF,float tau1FF, float tau2FF, float y1,float Pbias,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float u = -a1*oldj + b1*y1 + b2*oldFlowAvg;

  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (millis() - oldTimeControlRes)/1000;
  
  oldErrorRes = error;
  u = u + P + I;
  
  if (u > upperlimit) {u = upperlimit;}else{oldIntegralRes = I;}
  if (u < lowerlimit) {u = lowerlimit;}else{oldIntegralRes = I;}
  
  return u;
}

float feedForwardPIDPosition(float kc,float tauI,float tauD,float y,float ysp,float kFF,float tau1FF, float tau2FF, float y1,float Pbias,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float u = -a1*oldj + b1*y1 + b2*oldFlowAvg;

  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (millis() - oldTimeControlRes)/1000;
  oldIntegralRes = I;
  float D = kc * tauD * (error - oldErrorPump) / (millis() - oldTimeControlRes)/1000;
  oldErrorRes = error;
  u = u + P + I + D;
  
  if (u > upperlimit) {u = upperlimit;}else{oldIntegralRes = I;}
  if (u < lowerlimit) {u = lowerlimit;}else{oldIntegralRes = I;}
  
  return u;
}