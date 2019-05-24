/*
Anderson Vieira de Lima (andersonvieira14@gmail.com)
Waldney Souza de Andrade (waldney.andrade@gmail.com)
*/
#include <CustomStepper.h>

#define RPM_MAX 7
#define REV 4075.7728395
#define P 3.14159265359
//#define B 0.086
#define B 0.153
#define R 0.0325
#define MAX_COUNT 2147483647
#define K_HO 0.3
#define K_ALFA 2.0
#define K_BETA 0.0

// Constantes do caminho
#define N 21
#define X_I 0
#define Y_I 0
#define X_F 15
#define Y_F 1

struct No {
  int x;
  int y;
  struct No *prox;
};

CustomStepper mLeft(8, 9, 10, 11, (byte[]) {8, B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001}, REV, RPM_MAX, STOP);
//CustomStepper mLeft(11, 10, 9, 8, (byte[]) {8, B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001}, REV, RPM_MAX, STOP);
CustomStepper mRight(7, 6, 5, 4, (byte[]) {8, B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001}, REV, RPM_MAX, STOP);
//CustomStepper mRight(4, 5, 6, 7, (byte[]) {8, B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001}, REV, RPM_MAX, STOP);

int m[N][N];

float passosEsq = 0;
float passosDir = 0;
float deltaPassosEsq = 0;
float deltaPassosDir = 0;
float passosEsqAnt = 0;
float passosDirAnt = 0;
struct No* caminho;

unsigned long timeMover;
float x = 0;
float y = 0;
float teta = 0;
bool f = true;

float v_roda_d; //Velocidade da roda direita em m/s
float v_roda_e; //Velocidade da roda esquerda em m/s
float veld;
float vele;

float vel_lin;
float vel_ang;

float ho;

float calcVelLin(float ho);
float calcVelAng(float alfa, float beta);

float xatual(float xant, float ds, float q);
float yatual(float yant, float ds, float q);
float qatual(float qant, float dq);
float getDtetaTotal(float Dsd, float Dse);
float getDsTotal(float Dsd, float Dse);
float getDs(float Dq);
float getDteta(float NP);
void mover(double wD, double wE);
void att_localizacao();
float vd(float v, float w);
float ve(float v, float w);
float msToRpm(float vel);
struct No *manhattan(int m[][N]);
struct No *gerar_fila(struct No *fila, int m[][N], int x, int y);
struct No *path(struct No *fila, int m[][N], int x, int y);
struct No *inserir(struct No *fila, int x, int y);
struct No *remover(struct No *fila);
void imprimir(struct No *fila);

void setarray()
{
  for (int i=0;i<N;i++)
    for (int j=0;j<N;j++)
      m[i][j] = 1;
}

void setup() {
  //Econimia de bateria
  int i, j;
  for (byte i = 0; i <= A5; i++) {
    pinMode (i, OUTPUT);    // changed as per below
    digitalWrite (i, LOW);  
  }
    
  // disable ADC
  ADCSRA = 0; 
  
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  setarray();
  caminho = manhattan(m);
}

void loop() {
  if(f){
    x = caminho->x * 0.1;
    y = caminho->y * 0.1;
    f = false;
  }  
  //float deltaX = X_DEST - x;
  //float deltaY = Y_DEST - y;

  float deltaX = (caminho->prox->x * 0.1) - x;
  float deltaY = (caminho->prox->y * 0.1) - y;
  
  ho = sqrt( pow(deltaX, 2) + pow(deltaY, 2));
  float alfa = -1*teta + atan2(deltaY, deltaX);
  float beta = -1*teta - alfa;

  if (ho > 0.01) {

    vel_lin = calcVelLin(ho);
    vel_ang = calcVelAng(alfa, beta);

    v_roda_d = vd(vel_lin, vel_ang);
    v_roda_e = ve(vel_lin, vel_ang); 
    veld = msToRpm(v_roda_d);
    vele = msToRpm(v_roda_e);
    mover(veld, vele, 100);
    att_localizacao();
    Serial.print("\n rho: ");
    Serial.print(ho);
    Serial.print(" alfa: ");
    Serial.print(alfa);
    Serial.print(", x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.print(y);
    Serial.print(", vd: ");
    Serial.print(v_roda_d);
    Serial.print(", ve: ");
    Serial.print(v_roda_e);
  }else{
    if(caminho->prox->prox != NULL){
      f = true;
      caminho = remover(caminho);
      imprimir(caminho);
    }
  }
}

void mover(double wD, double wE, double t) {
  
  int rpmEsq = round(wE);
  int rpmDir = round(wD);

   
  if (abs(rpmEsq) > RPM_MAX) {
    if(rpmEsq > 0)  rpmEsq = RPM_MAX;
    else rpmEsq = -RPM_MAX;
  }
  if (abs(rpmDir) > RPM_MAX) {
    if(rpmDir > 0) rpmDir = RPM_MAX;
    else rpmDir = -RPM_MAX;
  }

    if (rpmEsq > 0) {
    mLeft.setDirection(CW);
    //Serial.print("Esq CW ");
  } else if(rpmEsq < 0){
    mLeft.setDirection(CCW);
    //Serial.print(" Esq CCW ");
  }
  
  if (rpmDir > 0) {
    mRight.setDirection(CW);
    Serial.print("Dir CW ");
  } else if(rpmDir < 0){
    mRight.setDirection(CCW);
    Serial.print("Dir CCW ");
  }

    Serial.print(", rpm_d: ");
    Serial.print(rpmDir);
    Serial.print(", rpm_e: ");
    Serial.print(rpmEsq);
  
  mLeft.setRPM(rpmEsq);
  mRight.setRPM(rpmDir);
  mLeft.rotate();
  mRight.rotate();
  
  timeMover = millis();
  while (millis() - timeMover < t) {
    mLeft.run();
    mRight.run();
  }
   
  mLeft.setDirection(STOP);
  mRight.setDirection(STOP);
  
}

void att_localizacao() {
  passosEsq = mLeft.getSteps();
  passosDir = mRight.getSteps();
  
  deltaPassosEsq = passosEsq - passosEsqAnt;
  deltaPassosDir = passosDir - passosDirAnt;

  passosEsqAnt = passosEsq;
  passosDirAnt = passosDir;
  
  // tratamento de descontinuidades motor esquerdo
  if (abs(deltaPassosEsq) > MAX_COUNT/2) {
    if (deltaPassosEsq < 0) {
      deltaPassosEsq = MAX_COUNT + deltaPassosEsq;
    } else {
      deltaPassosEsq = -MAX_COUNT + deltaPassosEsq;
    }
  }
  
  // tratamento de descontinuidades motor direito
  if (abs(deltaPassosDir) > MAX_COUNT/2) {
    if (deltaPassosDir < 0) {
      deltaPassosDir = MAX_COUNT + deltaPassosDir;
    } else {
      deltaPassosDir = -MAX_COUNT + deltaPassosDir;
    }
  }

  float dqe = getDteta(deltaPassosEsq);
  float dqd = getDteta(deltaPassosDir);

  float dse = getDs(dqe);
  float dsd = getDs(dqd);

  float dtetaTotal = getDtetaTotal(dsd, dse);
  float dsTotal = getDsTotal(dsd, dse);

  x = xatual(x, dsTotal, teta);
  y = yatual(y, dsTotal, teta);
  teta = qatual(teta, dtetaTotal);

  // deltaPassosDir me informa a quantidade de passos realizados por ciclo
  // deltaPassosEsq me informa a quantidade de passos realizados por ciclo
  
  // equacoes de odometria
  
  // modelo cinematico
 
  // atualizações
}

float xatual(float xant, float ds, float teta){
  return xant + (ds * cos(teta));
}

float yatual(float yant, float ds, float teta){
  return yant + (ds * sin(teta));
}

float qatual(float tetaant, float dteta){
  return tetaant + dteta;
}

float getDtetaTotal(float Dsd, float Dse) {
 return (Dsd - Dse)/B;
}

float getDsTotal(float Dsd, float Dse) {
 return (Dsd + Dse)/2;
}

float getDs(float Dq) {
  return R * Dq;
}

float getDteta(float NP){
  return ( 2 * P * NP) / REV;
}

float msToRpm(float vel){
  return (vel*60)/(2*P*R);
}

float vd(float v, float w){
  float vd = v + ((w*B)/2);
  return vd;  
}

float ve(float v, float w){
  float ve = v - ((w*B)/2);
  return ve; 
}

float calcVelLin(float ho) {
  return K_HO * ho;
}

float calcVelAng(float alfa, float beta) {
  return (K_ALFA * alfa) + (K_BETA * beta);
}

struct No *manhattan(int m[][N]){
    int i = X_F, j = Y_F, x, y;

  struct No *fila = NULL;
  struct No *aux;
    //int obj[22][2] = {{5 ,1},{6 ,1}, {7 ,1},{5 ,2}, {6 ,2}, {7 ,2},{5 ,3}, {6 ,3}, {7 ,3},{5 ,4}, {6 ,4}, {7 ,4}, {10, 7}, {11, 7}, {10, 8}, {11, 8}
    //                  , {10, 9}, {11, 9}, {10, 10}, {11, 10}, {10, 11}, {11, 11}};
    
  //for (i = 0; i <22; i++){
  //    int x = obj[i][0];
  //    int y = obj[i][1];
  //    m[x][y] = -1;
 // }


/*Montando a matriz inicial*/
for (i = 0; i < N; i++){
    for (j = 0; j < N; j++){
        /*Foi atribuido 1 à matriz pois o 0 gerou problemas ao ser comparado com o NULL*/
        m[i][j] = 1;
    }
}
/*As atribuições foram feitas manualmente pois ao usar matriz para representar os objetos o arduino (Uno) excedia a memória permitida*/
  m[4][0] = -1;
  m[5][0] = -1;
  m[6][0] = -1;
  m[7][0] = -1;
  m[8][0] = -1;
  m[9][0] = -1;
  m[10][0] = -1;
  m[11][0] = -1;
  m[12][0] = -1;
  m[13][0] = -1;
  m[4][1] = -1;
  m[5][1] = -1;
  m[6][1] = -1;
  m[7][1] = -1;
  m[8][1] = -1;
  m[9][1] = -1;
  m[10][1] = -1;
  m[11][1] = -1;
  m[12][1] = -1;
  m[13][1] = -1;
  m[4][2] = -1;
  m[5][2] = -1;
  m[6][2] = -1;
  m[7][2] = -1;
  m[8][2] = -1;
  m[9][2] = -1;
  m[10][2] = -1;
  m[11][2] = -1;
  m[12][2] = -1;
  m[13][2] = -1;
  m[4][3] = -1;
  m[5][3] = -1;
  m[6][3] = -1;
  m[7][3] = -1;
  m[8][3] = -1;
  m[9][3] = -1;
  m[10][3] = -1;
  m[11][3] = -1;
  m[12][3] = -1;
  m[13][3] = -1;
  m[4][4] = -1;
  m[5][4] = -1;
  m[6][4] = -1;
  m[7][4] = -1;
  m[8][4] = -1;
  m[9][4] = -1;
  m[10][4] = -1;
  m[11][4] = -1;
  m[12][4] = -1;
  m[13][4] = -1;
  m[4][5] = -1;
  m[5][5] = -1;
  m[6][5] = -1;
  m[7][5] = -1;
  m[8][5] = -1;
  m[9][5] = -1;
  m[10][5] = -1;
  m[11][5] = -1;
  m[12][5] = -1;
  m[13][5] = -1;
  m[4][6] = -1;
  m[5][6] = -1;
  m[6][6] = -1;
  m[7][6] = -1;
  m[8][6] = -1;
  m[9][6] = -1;
  m[10][6] = -1;
  m[11][6] = -1;
  m[12][6] = -1;
  m[13][6] = -1;

  m[8][8] = -1;
  m[9][8] = -1;
  m[10][8] = -1;
  m[11][8] = -1;
  m[12][8] = -1;
  m[13][8] = -1;
  m[14][8] = -1;
  m[15][8] = -1;
  m[8][9] = -1;
  m[9][9] = -1;
  m[10][9] = -1;
  m[11][9] = -1;
  m[12][9] = -1;
  m[13][9] = -1;
  m[14][9] = -1;
  m[15][9] = -1;
  m[8][10] = -1;
  m[9][10] = -1;
  m[10][10] = -1;
  m[11][10] = -1;
  m[12][10] = -1;
  m[13][10] = -1;
  m[14][10] = -1;
  m[15][10] = -1;
  m[8][11] = -1;
  m[9][11] = -1;
  m[10][11] = -1;
  m[11][11] = -1;
  m[12][11] = -1;
  m[13][11] = -1;
  m[14][11] = -1;
  m[15][11] = -1;
  m[8][12] = -1;
  m[9][12] = -1;
  m[10][12] = -1;
  m[11][12] = -1;
  m[12][12] = -1;
  m[13][12] = -1;
  m[14][12] = -1;
  m[15][12] = -1;
  m[8][13] = -1;
  m[9][13] = -1;
  m[10][13] = -1;
  m[11][13] = -1;
  m[12][13] = -1;
  m[13][13] = -1;
  m[14][13] = -1;
  m[15][13] = -1;
  m[8][14] = -1;
  m[9][14] = -1;
  m[10][14] = -1;
  m[11][14] = -1;
  m[12][14] = -1;
  m[13][14] = -1;
  m[14][14] = -1;
  m[15][14] = -1;
  m[8][15] = -1;
  m[9][15] = -1;
  m[10][15] = -1;
  m[11][15] = -1;
  m[12][15] = -1;
  m[13][15] = -1;
  m[14][15] = -1;
  m[15][15] = -1;
 
    /*Gera uma fila inicial da vizinhanзa e faz a incrementaзгo m[x][y] + 1*/
    fila = gerar_fila(fila, m, X_F, Y_F);
    
    /*Repete o processo atй que nгo restem elementos a serem mapeados, ou seja, o mapa estб completo*/
    while (fila != NULL) {
        fila = gerar_fila(fila, m, fila->x, fila->y);
        fila = remover(fila);
    }
    /*Atribui um valor especial а celula objetivo*/
    
    m[X_F][Y_F] = 1;

    /*for (i = 0; i < N; i++){
        for (j = 0; j < N; j++){
               Serial.print(m[i][j]);
               Serial.print(" ");
            
        }
        Serial.println(" ");
    }*/
  
    /*Gera o primeiro ponto a ser percorrido pelo robф*/
    fila = inserir(fila, X_I, Y_I);

    /*Ponteiro apontando para o inнcio da fila*/
    aux = fila;
    
    /*Adiciona elementos а fila de caminhos atй que o robo encontre o ponto objetivo*/
    while (fila->x != X_F || fila->y != Y_F) {
        fila = path(fila, m, fila->x, fila->y);
        fila = fila->prox;
    }
    /*Apontando para o inicio da fila novamente*/
    fila = aux;
    //imprimir(fila);
    return fila;
}

struct No *gerar_fila(struct No *fila, int m[][N], int x, int y){
    int i = x, j = y;
    if(m[i][j - 1] == 1 && (i >= 0 && i < N) && (j - 1 >= 0 && j - 1 < N)){
        m[i][j - 1] = m[i][j] + 1;
    fila = inserir(fila, i, j - 1);
    }
    if(m[i - 1][j] == 1 && (i - 1 >= 0 && i - 1 < N) && (j >= 0 && j < N)){
        m[i - 1][j] = m[i][j] + 1;
    fila = inserir(fila, i - 1, j);
    }
    if(m[i + 1][j] == 1 && (i + 1 >= 0 && i + 1 < N) && (j >= 0 && j < N)){
        m[i + 1][j] = m[i][j] + 1;
    fila = inserir(fila, i + 1, j);
    }

    
    if(m[i][j + 1] == 1 && (i >= 0 && i < N) && (j + 1 >= 0 && j + 1 < N)){
        m[i][j + 1] = m[i][j] + 1;
    fila = inserir(fila, i, j + 1);
    }

    return fila;
};

struct No *path(struct No *fila, int m[][N], int x, int y){
    int min = 1000;
    int i = x, j = y, X, Y;
    if((m[i - 1][j - 1] < min) && (m[i - 1][j - 1] > 0) && (i - 1 >= 0 && i - 1 < N) && (j - 1 >= 0 && j - 1 < N)){
        min = m[i - 1][j - 1];
        X = i - 1;
        Y = j - 1;
    }
    if((m[i][j - 1] < min) && (m[i][j - 1] > 0) && (i >= 0 && i < N) && (j - 1 >= 0 && j - 1 < N)){
        min = m[i][j - 1];
        X = i;
        Y = j - 1;
    }
    if((m[i + 1][j - 1] < min) && (m[i + 1][j - 1] > 0) && (i + 1 >= 0 && i + 1 < N) && (j - 1 >= 0 && j - 1 < N)){
        min = m[i + 1][j - 1];
        X = i + 1;
        Y = j - 1;
    }
    if((m[i - 1][j] < min) && (m[i - 1][j] > 0) && (i - 1 >= 0 && i - 1 < N) && (j >= 0 && j < N)){
        min = m[i - 1][j];
        X = i - 1;
        Y = j;
    }
    if ((m[i + 1][j] < min) && (m[i + 1][j] > 0) && (i + 1 >= 0 && i + 1 < N) && (j >= 0 && j < N)){
        min = m[i + 1][j];
        X = i + 1;
        Y = j;
    }

    if((m[i - 1][j + 1] < min) && (m[i - 1][j + 1] > 0) && (i - 1 >= 0 && i - 1 < N) && (j + 1 >= 0 && j + 1 < N)){
        min = m[i - 1][j + 1];
        X = i - 1;
        Y = j + 1;
    }
    if((m[i][j + 1] < min) && (m[i][j + 1] > 0) && (i >= 0 && i < N) && (j + 1 >= 0 && j + 1 < N)){
        min = m[i][j + 1];
        X = i;
        Y = j +  1;
    }

    if((m[i + 1][j + 1] < min) && (m[i + 1][j + 1] > 0) && (i + 1 >= 0 && i + 1 < N) && (j + 1 >= 0 && j + 1 < N)){
        min = m[i + 1][j + 1];
        X = i + 1;
        Y = j + 1;
    }
    fila = inserir(fila, X, Y);

    return fila;
}

struct No *inserir(struct No *fila, int x, int y){
  struct No *novo =(struct No*)malloc(sizeof(struct No));
  struct No *aux = fila;

  if(novo == NULL)
    return fila;
  novo->x = x;
  novo->y = y;
  if(fila == NULL){
    fila = novo;
    novo->prox = NULL;
    return fila;
  }

  while(aux->prox != NULL){
    //ant = aux;
    aux = aux->prox;
  }
  //novo->prox = fila;
  aux->prox = novo;
  novo->prox = NULL;
  return fila;
}

struct No *remover(struct No *fila){
  if(fila == NULL)
    return fila;
  struct No *aux = fila;
  fila = fila->prox;
  aux->prox = NULL;
  free(aux);
  return fila;
}

void imprimir(struct No *fila){
  if(fila == NULL)
    return ;
  struct No *aux = fila;

  do{
    //printf(" (%d, %d) ", aux->x, aux->y);
    Serial.print("(");
    Serial.print(aux->x);
    Serial.print(", ");
    Serial.print(aux->y);
    Serial.print(")");
    Serial.println("");
    aux = aux->prox;
  } while(aux!=NULL);

  printf("\n");
}
