#include <math.h>

#define WAVE_SQUARE_A A0 //模拟输入
#define WAVE_SQUARE_B A1 //模拟输入
#define WAVE_ORIGINAL_LIFTED_B A2 //用于B信号程控放大

#define WAVE_ORIGINAL_LIFTED_A A4 //模拟输入
#define WAVE_AMPLIFIED_LIFTED_B A5 //模拟输入

#define CD_CTRL_A 8 //数字输出
#define CD_CTRL_B 9 //数字输出

/*****************用于AB信号方波的频率和相位读取******************/
int wave_square_A = 0;
int jump_time_array_A[300] = {0};
int i_A = 0;
int flag_A = 1;
float phase_A = 0;
float frequency_A = 0;
int wave_square_B = 0;
int jump_time_array_B[300] = {0};
int i_B = 0;
int flag_B = 1;
float phase_B = 0;
float frequency_B = 0;

/*****************用于B信号的程控放大******************/
int wave_original_lifted_B = 0;
int wave_original_lifted_B_max[10] = {0};
int original_lifted_B_state_for_amplify = 0;

/*****************用于AB信号的实际数据实时读取******************/
int waveform_A = 0;
float wave_original_lifted_A = 0;
float wave_original_lifted_A_square_sum = 0;
// float wave_original_lifted_A_rms = 0;
float wave_original_lifted_A_rms_50 = 0;
int ii_A = 0;
int waveform_B = 0;
float wave_amplified_lifted_B = 0;
float wave_amplified_lifted_B_square_sum = 0;
// float wave_amplified_lifted_B_rms = 0;
float wave_amplified_lifted_B_rms_50 = 0;
int ii_B = 0;

/*****************用于存储AB信号的数据，用于液晶屏更新2Hz******************/
float phase_difference = 0; //相位差
float power_factor_cospsi = 0; //功率因数
float active_power = 0; //有功功率
float apparent_power = 0; //视在功率
float frequency_both = 0; //信号频率（此处取平均值）

/*****************用于存储屏幕串口发送给UNO的信号，以控制UNO传输状态******************/
unsigned char m_ReciverHeader;    //ATF Message Header char = @/#/$
unsigned short m_ReciverBodyID;   //ATF Message BodyID  0:GUI Swtich  100~65535:Body Msg
int m_ReciverVarInt;              //ATF Message int Val(Header=@)
float m_ReciverVarFloat;          //ATF Message float Val(Header=#)
String m_ReciverVarString;        //ATF Message String Val(Header=$)

/*****************用于时序控制******************/
long stamp_time = 0;
long last_time = 0;

/*****************用于UNO串口输出数据控制******************/
int TX_state = 0;

int TX_state_1_flag = 1;
int TX_state_2_flag = 1;
int TX_state_3_flag = 1;

/*****************用于求数组数据均值******************/
float get_array_avg(int array[],int num){
	int array_sum = 0;
	for (int i = 0; i < num; i++){
		array_sum = array_sum + array[i];
	}
	return float(array_sum/num);
}

/*****************用于清空数组数据******************/
void clear_array(int array[],int num){
	for (int i = 0; i < num; i++){
		array[i] = 0;
	}
	return;
}

/*****************用于求方波信号A/B的频率******************/
float get_frequency_avg(int jump_time_array[],int num){
	float period_time = (jump_time_array[num-1] - jump_time_array[0])/(num-1);
	return float(1/period_time)*1000;
}

// float get_mapped_realtime_stat(int PIN_NAME){
	// return map(analogRead(PIN_NAME),0,1023,0,5)
// }

/*****************用于获取信号B的挡位信息******************/
int get_B_state_for_amplify(){
	float original_lifted_B_max_avg = 0;
	while(1){
		if (millis()<5000){
			for (int j = 0; j < 10; j++){
				for (int i = 0; i <1000; i++){
          wave_original_lifted_B = analogRead(WAVE_ORIGINAL_LIFTED_B)*5;
					if (wave_original_lifted_B > wave_original_lifted_B_max[j]){
						wave_original_lifted_B_max[j] = wave_original_lifted_B;
            Serial.println(wave_original_lifted_B);
					}
				}
			}
			original_lifted_B_max_avg = get_array_avg(wave_original_lifted_B_max,10); // 计算最大值的均值
      Serial.println(original_lifted_B_max_avg);
			if (original_lifted_B_max_avg >= 0&& original_lifted_B_max_avg < 2655){return 3;} 		//10倍放大
			else if (original_lifted_B_max_avg >= 2655&& original_lifted_B_max_avg < 2760){return 2;}	//5倍放大
			else if (original_lifted_B_max_avg >= 2760&& original_lifted_B_max_avg < 2970){return 1;} //2.5倍放大
			// else if (original_lifted_B_max_avg >= 0.8&& original_lifted_B_max_avg < 2.5){return 0;} //1倍放大
			else{return 0;}
		}else{return 0;}
	}
}

/*****************用于读取屏幕串口传输的信息******************/
unsigned char ATFMessageService(unsigned char delaytimer)
{
	char n_TempChar;
		n_TempChar = Serial.available();
	if(n_TempChar){
		delay(delaytimer);
		n_TempChar = Serial.read();
		while(n_TempChar!='@'&&n_TempChar!='#'&&n_TempChar!='$'&&n_TempChar>=0){
			n_TempChar = Serial.read();
		}
		m_ReciverHeader = n_TempChar;
		m_ReciverBodyID = Serial.parseInt();
		if(n_TempChar=='@'){
			m_ReciverVarInt = Serial.parseInt();
		}
		else if(n_TempChar=='#'){
			m_ReciverVarFloat = Serial.parseFloat();
		}
		else if(n_TempChar=='$'){
			Serial.read();
			m_ReciverVarString = Serial.readStringUntil('\r'); 
		}
		else return 0;
		return 1;
	}
	return 0;
}

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	Serial.setTimeout(20);
	
	pinMode(CD_CTRL_A, OUTPUT);
	pinMode(CD_CTRL_B, OUTPUT);
	/*****************等待信号稳定******************/
	delay(2500);
	Serial.println("Ready1!");
	/*****************用于获取信号B的挡位信息，并控制CD4052进行选通******************/
	original_lifted_B_state_for_amplify = get_B_state_for_amplify();
	switch(original_lifted_B_state_for_amplify){
		case 0:digitalWrite(CD_CTRL_A,1);digitalWrite(CD_CTRL_B,1);break; // x1倍放大
		case 1:digitalWrite(CD_CTRL_A,0);digitalWrite(CD_CTRL_B,0);break; // x2.5倍放大
		case 2:digitalWrite(CD_CTRL_A,1);digitalWrite(CD_CTRL_B,0);break; // x5倍放大
		case 3:digitalWrite(CD_CTRL_A,0);digitalWrite(CD_CTRL_B,1);break; // x10倍放大
		default:digitalWrite(CD_CTRL_A,0);digitalWrite(CD_CTRL_B,0);break;
	}
	/*****************等待信号稳定******************/
	delay(2500);
	Serial.print("Ready2! Mode Select: ");
	Serial.println(original_lifted_B_state_for_amplify);
}

void loop() {
	// put your main code here, to run repeatedly:
	stamp_time = millis();
	if(last_time != stamp_time){
		last_time = stamp_time;
		/*****************用于AB信号的频率与相位读取******************/
		wave_square_A = analogRead(WAVE_SQUARE_A);
		wave_square_B = analogRead(WAVE_SQUARE_B);
		/*****************方波信号A跳变检测******************/
		if (wave_square_A > 222 && flag_A == 1){
			if (i_A < 300){
				jump_time_array_A[i_A] = stamp_time;
				i_A++; flag_A = 0;
			}else{
				i_A = 0; flag_A = 1;
				phase_A = get_array_avg(jump_time_array_A,300);
				frequency_A = get_frequency_avg(jump_time_array_A,300);
				clear_array(jump_time_array_A,300);
			}
		}else if (wave_square_A < 222 && flag_A == 0){
			flag_A = 1;
		}
		/*****************方波信号B跳变检测******************/
		if (wave_square_B > 222 && flag_B == 1){
			if (i_B < 300){
				jump_time_array_B[i_B] = stamp_time;
				i_B++; flag_B = 0;
			}else{
				i_B = 0; flag_B = 1;
				phase_B = get_array_avg(jump_time_array_B,300);
				frequency_B = get_frequency_avg(jump_time_array_B,300);
				clear_array(jump_time_array_B,300);
			}
		}else if (wave_square_B < 222 && flag_B == 0){
			flag_B = 1;
		}

		
		/*****************用于AB信号的数据获取******************/
		wave_original_lifted_A = analogRead(WAVE_ORIGINAL_LIFTED_A)*0.488-250; //映射到220V，并减去2.5V偏置
		wave_amplified_lifted_B = analogRead(WAVE_AMPLIFIED_LIFTED_B)*0.488-250; //映射到100A，并减去2.5V偏置
		wave_original_lifted_A = abs(wave_original_lifted_A);
		wave_amplified_lifted_B = abs(wave_amplified_lifted_B);
		/*****************信号A有效值获取，用于计算功率相关数据******************/
		if (ii_A < 1296){
			wave_original_lifted_A_square_sum = wave_original_lifted_A_square_sum + wave_original_lifted_A*wave_original_lifted_A;
			ii_A++;
		}else{
			ii_A = 0;
			wave_original_lifted_A_rms_50 = sqrt(wave_original_lifted_A_square_sum)/36;
			/*****************方波信号AB功率相关参数计算******************/
			apparent_power = wave_original_lifted_A_rms_50*wave_amplified_lifted_B_rms_50;
			active_power = apparent_power*power_factor_cospsi;
			wave_original_lifted_A_square_sum = 0;
		}
		/*****************信号B有效值获取，用于计算功率相关数据1Hz******************/
		if (ii_B < 1296){
			wave_amplified_lifted_B_square_sum = wave_amplified_lifted_B_square_sum + wave_amplified_lifted_B*wave_amplified_lifted_B;
			ii_B++;
		}else{
			ii_B = 0;
			wave_amplified_lifted_B_rms_50 = sqrt(wave_amplified_lifted_B_square_sum)/36;
			switch(original_lifted_B_state_for_amplify){
				case 0:wave_amplified_lifted_B_rms_50 = wave_amplified_lifted_B_rms_50;break; // x1倍放大
				case 1:wave_amplified_lifted_B_rms_50 = wave_amplified_lifted_B_rms_50/2.5;break; // x2.5倍放大
				case 2:wave_amplified_lifted_B_rms_50 = wave_amplified_lifted_B_rms_50/5;break; // x5倍放大
				case 3:wave_amplified_lifted_B_rms_50 = wave_amplified_lifted_B_rms_50/10;break; // x10倍放大
				default:wave_amplified_lifted_B_rms_50 = wave_amplified_lifted_B_rms_50;break;
			}
			/*****************方波信号AB功率相关参数计算******************/
			apparent_power = wave_original_lifted_A_rms_50*wave_amplified_lifted_B_rms_50;
			active_power = apparent_power*power_factor_cospsi;
			wave_amplified_lifted_B_square_sum = 0;
		}
		
		if(stamp_time%500 == 0){
			/*****************方波信号AB相位差计算及频率计算******************/
			float phase_difference_sub = phase_A-phase_B;
			frequency_both = (frequency_A + frequency_B)/2;
			phase_difference = phase_difference_sub*frequency_both*6.283/6000;
			power_factor_cospsi = cos(phase_difference);
			power_factor_cospsi = abs(power_factor_cospsi);
			if(power_factor_cospsi<0.5){power_factor_cospsi = 1-power_factor_cospsi;}
		}
		
		/*****************用于从屏幕获取当前状态******************/
		if(ATFMessageService(1)){
			if(m_ReciverBodyID==100 && m_ReciverVarInt==1){
				TX_state = 1; //波形A界面输出
			}else if(m_ReciverBodyID==101 && m_ReciverVarInt==1){
				TX_state = 2; //波形B界面输出
			}else if(m_ReciverBodyID==102 && m_ReciverVarInt==1){
				TX_state = 3; //波形AB界面输出
			}else if((m_ReciverBodyID==104 || m_ReciverBodyID==114 || m_ReciverBodyID==120) && m_ReciverVarInt==1){
				TX_state = 0; //退出波形输出界面
			}
		}
		
		/*****************用于控制向屏幕传输数据******************/
		if(stamp_time%1000 == 0){
			if(TX_state == 1){ //波形A界面输出
				TX_state_1_flag = 1;

			}else if(TX_state == 2){ //波形B界面输出
				TX_state_2_flag = 1;
				
			}else if(TX_state == 3){ //波形AB界面输出
				TX_state_3_flag = 1;
			}else{}
		}
		
		/*****************用于向屏幕传输波形数据******************/
		if(stamp_time%81 == 0){
			waveform_A = map(analogRead(WAVE_ORIGINAL_LIFTED_A),0,1023,0,139);
			waveform_B = map(analogRead(WAVE_AMPLIFIED_LIFTED_B),0,1023,-70,69)*2+70;
			if(TX_state == 1){ //波形A界面输出
				/****A信号波形****/
				Serial.print("@SET 113,");
				Serial.println(waveform_A);
				if(TX_state_1_flag){
					TX_state_1_flag = 0;
					/****A信号频率****/
					Serial.print("@SET 115,");
					Serial.println(frequency_A,2);
					/****A信号有效值****/
					Serial.print("@SET 116,");
					Serial.println(wave_original_lifted_A_rms_50,2);
					
				}
			}else if(TX_state == 2){ //波形B界面输出
				/****B信号波形****/
				Serial.print("@SET 119,");
				Serial.println(waveform_B);
				if(TX_state_2_flag){
					TX_state_2_flag = 0;
					/****B信号频率****/
					Serial.print("@SET 121,");
					Serial.println(frequency_A,2);
					/****B信号有效值****/
					Serial.print("@SET 122,");
					Serial.println(wave_amplified_lifted_B_rms_50,2);
					
				}
			}else if(TX_state == 3){ //波形AB界面输出
				/****AB信号波形****/
				Serial.print("@SET 103,");
				Serial.print(waveform_A);
				Serial.print("/");
				Serial.println(waveform_B);
				if(TX_state_3_flag){
					TX_state_3_flag = 0;
					/****AB信号频率****/
					Serial.print("@SET 109,");
					Serial.println(frequency_A,2);
					/****功率因数****/
					Serial.print("@SET 110,");
					Serial.println(power_factor_cospsi,2);
					/****视在功率****/
					Serial.print("@SET 112,");
					Serial.println(apparent_power,2);
					/****有功功率****/
					Serial.print("@SET 111,");
					Serial.println(active_power,2);	
				}
			}else{}
		}
	}
}