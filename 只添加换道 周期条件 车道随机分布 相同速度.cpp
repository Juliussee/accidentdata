//#define _CRT_SECURE_NO_WARNINGS

//#include <bits/stdc++.h>
#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
#include<fstream>
#include<cmath>
#include<ctime>
#include<string>
#include<cstdlib>
#include<windows.h>

#define road_long 10000		//路长（格）
int LB = 45;						//这里修改5  6  10  20     5*9
double Qin ;				//主干道入车概率

#define L_cell 1.5			//一个元胞实际长度(m) 
#define total_timestep 50000
#define count_timestep 10000
int i, j, k, veh_sum_save, randnumber, x, e;
double occupancy;
double density;

int time_step;
int veh_init_gap;  //道路初始化间距

//主干道模型

int veh_sum_left;				//左车道车辆总数
double vel_sum_left;				//左车道速度和
int veh_sum_right;				//右车道车辆总数
double vel_sum_right;				//右车道速度和
double density_sum_left;
double density_sum_right;
double flow_sum_left;
double flow_sum_right;

int sam=1;
int sample=10;
int outNum = 0;//记录出车数
int coil_time_gap = 60; //线圈输出间隔
int LB_change_num = 0;
int velocity_sume_left = 0;//统计速度
int velocity_sume_right = 0;
int velsum_rone;
int velsum_left;
int velsum_right;
double sample_velsum_left[20];
double sample_velsum_right[20];
double sample_velsum_rone[20];

double r_pro;//长车和短车的比例
int vmax_short=20;
int vmax_long=20;//两车速度

int vleft_save[road_long]= {-1}; //保存速度
int vright_save[road_long]= {-1};

int h_con=6;
int gapsafe=7;
int vanti;
double p_slow;
double pb=0.94;//此为慢化概率
double p0=0.5;
double pd=0.1;
double th_left;
double ts_left;
double th_right;
double ts_right;
double tc=10;
int veh_length_short= 5;		//车长（格
int veh_length_long= 12;		//车长（格）

int vmax;
int veh_length;
int fortemp;

//线圈输出
int ground_coil_position=5000;//线圈统计位置
int coil_veh_sum_right = 0;
int coil_velosity_sum_right = 0;
int coil_veh_sum_left = 0;
int coil_velosity_sum_left = 0;

typedef struct car_left {
	int x;
	int v;
	int vob;
	int gap;
	int gap_other;
	int gap_back;
	int vpred;//相邻车道前车速度
	int pred_gap;//相邻车道前车间距
	int position;
	int isinsert;//代表车辆是否可插入
	int type;//0表示短车，1表示长车
	int breaksta;
	int tst;//表示车辆停止时间
	int tarb;//表示相邻车道前车刹车灯状态
} CARL;

typedef struct car_right {
	int x;
	int v;
	int vob;
	int gap;
	int gap_other;
	int gap_back;
	int vpred;
	int pred_gap;
	int position;
	int isinsert;
	int type;//0表示短车，1表示长车
	int breaksta;
	int tst;//表示车辆停止时间
	int tarb;
} CARR;

//两个数产生最小值
int getMin2(int m, int n) {
	if (m < n) {
		return m;
	} else {
		return n;
	}
}

//两个数产生最大值
int getMax2(int m, int n) {
	if (m < n) {
		return n;
	} else {
		return m;
	}
}

//三个数求最小值
int getMin3(int u, int m, int z) {
	int  b;
	if (u > z) {
		if (z > m) {
			b = m;
		} else {
			b = z;
		}
	} else if (u < m) {
		b = u;
	} else {
		b = m;
	}
	return b;
}


//主干道撒车
void roadRandomInitialize(CARL* car_left, CARR* car_right) {

	int veh_sum_long=(road_long*occupancy*r_pro)*2/(7*r_pro+5);//双车道长车总数目

	int veh_sum_short=(1-r_pro)*(road_long*occupancy)*2/(7*r_pro+5); //双车道短车总数目

	//printf("占有率为：%f 比例为：%f 长车数目为：%d 短车数目为：%d 总车辆数为：%d\n\n", occupancy, r_pro, veh_sum_long, veh_sum_short, veh_sum_long + veh_sum_short);

	int falsenum=0;//记录插入失败次数
	int isf;//标记是否失败

	int isoccupy_left[10000]= {0}; //随机撒车表示被占据的格点位置
	int islong_left[10000]= {0}; //该点有车，且为大车
	int isoccupy_right[10000]= {0}; //随机撒车表示被占据的格点位置
	int islong_right[10000]= {0}; //该点有车，且为大车

	i=0;
	while(i<veh_sum_long) { //先撒长车
		int ramposi=rand()%9989;//在0-9989位置上随机撒一辆车
		isf=0; //标记是否失败

//		if(falsenum>200000){
//			printf("当前插入不了");
//		}

		if(((double)rand())/RAND_MAX<0.5) {//随机在左车道

			for(j=ramposi-11; j<ramposi+12; j++) {
				if(isoccupy_left[j]==1) {
					isf=1;
					break;//这里表示退出当前循环
				}
			}

			if(isf==1) {
				falsenum++;
				continue;//若当前插入失败，重新插入
			}

			isoccupy_left[ramposi]=1;//这里撒车成功
			islong_left[ramposi]=1;//该点为大车
			i++;
			//printf("车辆数目：%d 车辆位置：%d 失败次数：%d\n",i,ramposi,falsenum);

		} else {//随机在右车道

			for(j=ramposi-11; j<ramposi+12; j++) {
				if(isoccupy_right[j]==1) {
					isf=1;
					break;//这里表示退出当前循环
				}
			}

			if(isf==1) {
				falsenum++;
				continue;//若当前插入失败，重新插入
			}

			isoccupy_right[ramposi]=1;//这里撒车成功
			islong_right[ramposi]=1;//该点为大车
			i++;
			//printf("车辆数目：%d 车辆位置：%d 失败次数：%d\n",i,ramposi,falsenum);
		}


	}

	i=0;
	while(i<veh_sum_short) { //先撒短车
		int ramposi=rand()%9996;//在0-9989位置上随机撒一辆车

		isf=0; //标记是否失败

		if(((double)rand())/RAND_MAX<0.5) {//随机在左车道

			for(j=ramposi-11; j<ramposi+5; j++) {//这里-4会发生错误
				if(islong_left[j]==1||j>ramposi-5&&isoccupy_left[j]==1) { //在 ramposi-11到ramposi-5之间有大车，或者 ramposi-5以后有小车
					isf=1;
					break;//这里表示退出当前循环
				}
			}

			if(isf==1) {
				falsenum++;
				continue;//若当前插入失败，重新插入
			}

			isoccupy_left[ramposi]=1;//这里撒车成功
			i++;
			//printf("车辆数目：%d 车辆位置：%d 失败次数：%d\n",i,ramposi,falsenum);

		} else {

			for(j=ramposi-11; j<ramposi+5; j++) {
				if(islong_right[j]==1||j>ramposi-5&&isoccupy_right[j]==1) { //在 ramposi-11到ramposi-5之间有大车，或者 ramposi-5以后有小车
					isf=1;
					break;//这里表示退出当前循环
				}
			}

			if(isf==1) {
				falsenum++;
				continue;//若当前插入失败，重新插入
			}

			isoccupy_right[ramposi]=1;//这里撒车成功
			i++;
			//printf("车辆数目：%d 车辆位置：%d 失败次数：%d\n",i,ramposi,falsenum);
		}

	}

	for(i=road_long-1; i>=0; i--) {
		if(isoccupy_left[i]==1) {
			car_left[veh_sum_left].x=i;// veh_sum_left从第0辆开始递增，这里i表示位置
			car_left[veh_sum_left].v=0;
			if(islong_left[i]==1) {
				car_left[veh_sum_left].type= 1;
			} else {
				car_left[veh_sum_left].type= 0;
			}

			//printf("当前左车道车辆数目为：%d 位置：%d 类型：%d\n",veh_sum_left+1,car_left[veh_sum_left].x,car_left[veh_sum_left].type) ;

			veh_sum_left++ ;

		}
	}

	for(i=road_long-1; i>=0; i--) {
		if(isoccupy_right[i]==1) {
			car_right[veh_sum_right].x=i;// veh_sum_left从第0辆开始递增，这里i表示位置
			car_right[veh_sum_right].v=0;
			if(islong_right[i]==1) {
				car_right[veh_sum_right].type= 1;
			} else {
				car_right[veh_sum_right].type= 0;
			}

			//printf("当前右车道车辆数目为：%d 位置：%d 类型：%d\n",veh_sum_right+1,car_right[veh_sum_right].x,car_right[veh_sum_right].type) ;

			veh_sum_right++ ;

		}
	}

	for (i = 0; i < veh_sum_left; i++) {

		if(car_left[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_left[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("左车道当前时步：%d 还有别的类型\n",time_step);
		}

		if (i == 0) {//最前面一辆车子
			car_left[i].gap = car_left[veh_sum_left - 1].x - car_left[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if (car_left[i].gap < 0) {
			printf("左撒车错误 时步：%d 车辆总数：%d 编号：%d 位置：%d 类型：%d 间距：%d 前车位置：%d\n",time_step,veh_sum_left,i,car_left[i].x,car_left[i].type,car_left[i].gap,car_left[i-1].x);
		}

	}

	for (i = 0; i < veh_sum_right; i++) {

		if(car_right[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_right[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("右车道当前时步：%d 还有别的类型 编号：%d 类型：%d \n",time_step,i,car_right[i].type);
		}

		if (i == 0) {//最前面一辆车子
			car_right[i].gap = car_right[veh_sum_right - 1].x - car_right[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if (car_right[i].gap < 0) {
			printf("右撒车错误 时步：%d 车辆总数:%d 编号：%d 位置：%d 类型：%d 间距：%d 前车位置：%d\n",time_step,veh_sum_right,i,car_right[i].x,car_right[i].type,car_right[i].gap,car_right[i-1].x);
		}


	}

	printf("撒车结束，左车道车辆总数：%d 右车道车辆总数：%d 失败次数：%d\n",veh_sum_left,veh_sum_right,falsenum) ;


}


/*
tep1:计算gap_other,gap_back,gap,vob分别表示相邻车道前后距离，和当前车与前车间距,另一车道跟随车距离
*step2:找出所有符合换道规则的车辆
*step3:车辆换道
*step4：左右车道进行NS规则变换，左车道可出车，右车道不可。
*/


int velocitySum(CARL* car_left, CARR* car_right) {
	//格点初始化(换道之前统计格点):
	for (i = 0; i < 10200; i++) {
		car_left[i].position = 0;
		car_right[i].position = 0;
	}

	//进行初始化操作
	for (i = 0; i < veh_sum_left; i++) {
		car_left[i].gap_back = -1;
		car_left[i].gap_other = -1;
		car_left[i].tarb = 0;
		car_left[car_left[i].x].position = 1;
		car_left[i].vob = 9999;


	}
	for (i = 0; i < veh_sum_right; i++) {
		car_right[i].gap_back = -1;
		car_right[i].gap_other = -1;
		car_right[i].tarb = 0;
		car_right[car_right[i].x].position = 1;
		car_right[i].vob = 9999;
	}

	/*step1*/
	/*计算左车道gap_other, gap_back, gap, vob*/

	fortemp=0;//for循环改进地方

	for (i = 0; i < veh_sum_left; i++) {

		if(car_left[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_left[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("左车道当前时步：%d 还有别的类型\n",time_step);
		}

		if (i == 0) {//最前面一辆车子
			car_left[i].gap = car_left[veh_sum_left - 1].x - car_left[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if (car_left[i].gap < 0) {
			printf("左1错误 时步：%d 车辆总数：%d 编号：%d 位置：%d 类型：%d 间距：%d 前车位置：%d\n",time_step,veh_sum_left,i,car_left[i].x,car_left[i].type,car_left[i].gap,car_left[i-1].x);
		}

		if (car_left[i].x > car_right[0].x) {//特殊情形1：在0和sum-1之间，即当前车在相邻车道第0辆车前面
			car_left[i].gap_other = car_right[veh_sum_right-1].x + road_long - car_left[i].x - veh_length;//相邻车道最后一辆车+road-当前车间距-车长

			if(car_right[0].type==0) {
				car_left[i].gap_back = car_left[i].x - car_right[0].x - veh_length_short;//这里以车尾计算间距，则需要减去的是后车的车辆长度
			} else {
				car_left[i].gap_back = car_left[i].x - car_right[0].x - veh_length_long;//相邻车道后车为长车
			}

			car_left[i].vob = car_right[0].v;
			car_left[i].vpred= car_right[veh_sum_right-1].v;
			car_left[i].tarb=car_right[veh_sum_right-1].breaksta;

			if(car_right[veh_sum_right-1].type==0) {
				car_left[i].pred_gap=car_right[veh_sum_right-2].x-car_right[veh_sum_right-1].x-veh_length_short;//短车
			} else {
				car_left[i].pred_gap=car_right[veh_sum_right-2].x-car_right[veh_sum_right-1].x-veh_length_long;//短车
			}


		} else if(car_left[i].x<car_right[veh_sum_right-1].x) { //特殊情况：车辆在相邻车道最后一辆车后面
			car_left[i].gap_other = car_right[veh_sum_right-1].x- car_left[i].x - veh_length;

			if(car_right[0].type==0) {
				car_left[i].gap_back = car_left[i].x + road_long - car_right[0].x - veh_length_short;//这里以车尾计算间距，则需要减去的是后车的车辆长度
			} else {
				car_left[i].gap_back = car_left[i].x + road_long - car_right[0].x - veh_length_long;//相邻车道后车为长车
			}

			car_left[i].vob = car_right[0].v;
			car_left[i].vpred= car_right[veh_sum_right-1].v;
			car_left[i].tarb= car_right[veh_sum_right-1].breaksta;

			if(car_right[veh_sum_right-1].type==0) {
				car_left[i].pred_gap=car_right[veh_sum_right-2].x-car_right[veh_sum_right-1].x-veh_length_short;//短车
			} else {
				car_left[i].pred_gap=car_right[veh_sum_right-2].x-car_right[veh_sum_right-1].x-veh_length_long;//长车
			}

		} else if(car_left[i].x<car_right[0].x&&car_left[i].x >=car_right[1].x) { //情况2：在01车之间
			car_left[i].gap_other = car_right[0].x - car_left[i].x - veh_length;

			if(car_right[1].type==0) {
				car_left[i].gap_back = car_left[i].x - car_right[1].x - veh_length_short;//这里以车尾计算间距，则需要减去的是后车的车辆长度
			} else {
				car_left[i].gap_back = car_left[i].x - car_right[1].x - veh_length_long;//相邻车道后车为长车
			}

			car_left[i].vob = car_right[1].v;
			car_left[i].vpred= car_right[0].v;
			car_left[i].tarb= car_right[0].breaksta;

			if(car_right[0].type==0) {
				car_left[i].pred_gap=car_right[veh_sum_right-1].x+road_long-car_right[0].x-veh_length_short;//短车
			} else {
				car_left[i].pred_gap=car_right[veh_sum_right-1].x+road_long-car_right[0].x-veh_length_long;//短车
			}

		} else {//一般情况

			//一般情况改进
			while(car_right[fortemp].x>car_left[i].x) {
				fortemp++;
			}
			if(car_right[fortemp].x==car_left[i].x) {
				car_left[i].gap_back = -1;
				car_left[i].gap_other = -1;
				car_left[i].vob = 9999;
				car_left[i].vpred = -1;
				car_left[i].tarb=-1;
				car_left[i].pred_gap=0;
			} else {

				if(car_right[fortemp].x>car_left[i].x||car_right[fortemp-1].x<car_left[i].x) {
					printf("当前fortemp计算发生错误");
				}

				car_left[i].gap_other = car_right[fortemp-1].x - car_left[i].x - veh_length;

				if(car_right[fortemp].type==0) {
					car_left[i].gap_back = car_left[i].x - car_right[fortemp].x - veh_length_short;
				} else {
					car_left[i].gap_back = car_left[i].x - car_right[fortemp].x - veh_length_long;
				}

				car_left[i].vob = car_right[fortemp].v;
				car_left[i].vpred= car_right[fortemp-1].v;
				car_left[i].tarb= car_right[fortemp-1].breaksta;

				if(car_right[fortemp-1].type==0) {
					car_left[i].pred_gap=car_right[fortemp-2].x-car_right[fortemp-1].x-veh_length_short;//考虑相邻两车道前车的类型
				} else {
					car_left[i].pred_gap=car_right[fortemp-2].x-car_right[fortemp-1].x-veh_length_long;//考虑相邻两车道前车的类型
				}


			}

		}
	}


	/*计算右车道gap_other, gap_back, gap, vob*/
	fortemp=0;//for循环改进地方
	for (i = 0; i < veh_sum_right; i++) {

		if(car_right[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_right[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("右车道当前时步：%d 还有别的类型 编号：%d 类型：%d \n",time_step,i,car_right[i].type);
		}

		if (i == 0) {//最前面一辆车子
			car_right[i].gap = car_right[veh_sum_right - 1].x - car_right[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if (car_right[i].gap < 0) {
			printf("右1错误 时步：%d 车辆总数:%d 编号：%d 位置：%d 类型：%d 间距：%d 前车位置：%d\n",time_step,veh_sum_right,i,car_right[i].x,car_right[i].type,car_right[i].gap,car_right[i-1].x);
		}

		if (car_right[i].x > car_left[0].x) {//特殊情形1：在0和sum-1之间，即当前车在相邻车道第0辆车前面
			car_right[i].gap_other = car_left[veh_sum_left-1].x + road_long - car_right[i].x - veh_length;

			if(car_left[0].type==0) {
				car_right[i].gap_back = car_right[i].x - car_left[0].x - veh_length_short;
			} else {
				car_right[i].gap_back = car_right[i].x - car_left[0].x - veh_length_long;
			}

			car_right[i].vob = car_left[0].v;
			car_right[i].vpred=car_left[veh_sum_left-1].v;
			car_right[i].tarb=car_left[veh_sum_left-1].breaksta;

			if(car_left[veh_sum_left-1].type==0) {
				car_right[i].pred_gap=car_left[veh_sum_left-2].x-car_left[veh_sum_left-1].x-veh_length_short;
			} else {
				car_right[i].pred_gap=car_left[veh_sum_left-2].x-car_left[veh_sum_left-1].x-veh_length_long;
			}

		} else if (car_right[i].x<car_left[veh_sum_left-1].x) { //车辆在相邻车道最后一辆车的后面
			car_right[i].gap_other = car_left[veh_sum_left-1].x - car_right[i].x - veh_length;

			if(car_left[0].type==0) {
				car_right[i].gap_back = car_right[i].x + road_long - car_left[0].x - veh_length_short;
			} else {
				car_right[i].gap_back = car_right[i].x + road_long - car_left[0].x - veh_length_long;
			}

			car_right[i].vob = car_left[0].v;
			car_right[i].vpred=car_left[veh_sum_left-1].v;
			car_right[i].tarb=car_left[veh_sum_left-1].breaksta;

			if(car_left[veh_sum_left-1].type==0) {
				car_right[i].pred_gap=car_left[veh_sum_left-2].x-car_left[veh_sum_left-1].x-veh_length_short;
			} else {
				car_right[i].pred_gap=car_left[veh_sum_left-2].x-car_left[veh_sum_left-1].x-veh_length_long;
			}

		} else if(car_right[i].x<car_left[0].x&&car_right[i].x>=car_left[1].x) { //位于01车两者之间
			car_right[i].gap_other = car_left[0].x - car_right[i].x - veh_length;

			if(car_left[1].type==0) {
				car_right[i].gap_back = car_right[i].x - car_left[1].x - veh_length_short;
			} else {
				car_right[i].gap_back = car_right[i].x - car_left[1].x - veh_length_long;
			}

			car_right[i].vob = car_left[1].v;
			car_right[i].vpred=car_left[0].v;
			car_right[i].tarb=car_left[0].breaksta;

			if(car_left[0].type==0) {
				car_right[i].pred_gap = car_left[veh_sum_left-1].x + road_long - car_left[0].x - veh_length_short;
			} else {
				car_right[i].pred_gap = car_left[veh_sum_left-1].x + road_long - car_left[0].x - veh_length_long;
			}

		} else {
			//一般情况改进
			while(car_left[fortemp].x>car_right[i].x) {
				fortemp++;
			}
			if(car_left[fortemp].x==car_right[i].x) {
				car_right[i].gap_back = -1;
				car_right[i].gap_other = -1;
				car_right[i].tarb = -1;
				car_right[i].vob = 9999;
				car_right[i].vpred = -1;
				car_right[i].pred_gap=0;
			} else {

				if(car_left[fortemp].x>car_right[i].x||car_left[fortemp-1].x<car_right[i].x) {
					printf("当前fortemp计算发生错误");
				}

				car_right[i].gap_other = car_left[fortemp-1].x - car_right[i].x - veh_length;

				if(car_left[fortemp].type==0) {
					car_right[i].gap_back = car_right[i].x - car_left[fortemp].x - veh_length_short;
				} else {
					car_right[i].gap_back = car_right[i].x - car_left[fortemp].x - veh_length_long;
				}

				car_right[i].vob = car_left[fortemp].v;
				car_right[i].vpred= car_left[fortemp-1].v;
				car_right[i].tarb=car_left[fortemp-1].breaksta;

				if(car_left[fortemp-1].type==0) {
					car_right[i].pred_gap=car_left[fortemp-2].x-car_left[fortemp-1].x-veh_length_short;//考虑相邻两车道前车的类型
				} else {
					car_right[i].pred_gap=car_left[fortemp-2].x-car_left[fortemp-1].x-veh_length_long;//考虑相邻两车道前车的类型
				}
			}
		}
	}


	/*step2*/
	/*判断车道需要换道的车辆*/
	for (i = 0; i < veh_sum_left; i++) {
		if(car_left[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else {
			vmax =vmax_long;
			veh_length=veh_length_long;
		}

		//换道模型改进
		int d_pred_left=car_left[i].gap_other+getMax2(0,getMin2(car_left[i].pred_gap,car_left[i].vpred)-gapsafe);

		int diff_left=getMin2(car_left[i].vob+1,vmax)-getMin2(car_left[i].x+1,vmax);

		if(i==0) {
			if(car_left[veh_sum_left-1].tarb==0) {//相邻车道前车刹车灯状况
				if(car_left[i].gap_other>=car_left[i].gap+car_left[i].v&& car_left[i].gap<getMin2(car_left[i].v+1,vmax)&&car_left[i].gap_back>=car_left[i].vob+diff_left) {
					car_left[i].isinsert = 1;
				} else {
					car_left[i].isinsert = 0;
				}
			} else {
				if (car_left[i].gap<getMin2(car_left[i].v+1,vmax) && d_pred_left >= car_left[i].gap+ car_left[i].v&&car_left[i].gap_back>=car_left[i].vob+diff_left) {
					car_left[i].isinsert = 1;
				} else {
					car_left[i].isinsert = 0;
				}
			}
		} else {
			if(car_left[i-1].tarb==0) {
				if(car_left[i].gap_other>=car_left[i].gap+car_left[i].v&& car_left[i].gap<getMin2(car_left[i].v+1,vmax)&&car_left[i].gap_back>=car_left[i].vob+diff_left) {
					car_left[i].isinsert = 1;
				} else {
					car_left[i].isinsert = 0;
				}
			} else {
				if (car_left[i].gap<getMin2(car_left[i].v+1,vmax) && d_pred_left >= car_left[i].gap+ car_left[i].v&&car_left[i].gap_back>=car_left[i].vob+diff_left) {
					car_left[i].isinsert = 1;
				} else {
					car_left[i].isinsert = 0;
				}
			}
		}


	}


	for (i = 0; i < veh_sum_right; i++) {
		if(car_right[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else {
			vmax =vmax_long;
			veh_length=veh_length_long;
		}

		//换道模型改进
		int d_pred_right=car_right[i].gap_other+getMax2(0,getMin2(car_right[i].vpred,car_right[i].pred_gap)-gapsafe);

		int diff_right=getMin2(car_right[i].vob+1,vmax)-getMin2(car_right[i].x+1,vmax);

		if(i==0) {
			if(car_right[veh_sum_right-1].tarb==0) {
				if(car_right[i].gap_other>=car_right[i].gap+car_right[i].v&& car_right[i].gap<getMin2(car_right[i].v+1,vmax)&&car_right[i].gap_back>=car_right[i].vob+diff_right) {
					car_right[i].isinsert = 1;
				} else {
					car_right[i].isinsert = 0;
				}
			} else {
				if (car_right[i].gap<getMin2(car_right[i].v+1,vmax) && d_pred_right >= car_right[i].gap+ car_right[i].v&&car_right[i].gap_back>=car_right[i].vob+diff_right) {
					car_right[i].isinsert = 1;
				} else {
					car_right[i].isinsert = 0;
				}
			}
		} else {
			if(car_right[i-1].tarb==0) {
				if(car_right[i].gap_other>=car_right[i].gap+car_right[i].v&& car_right[i].gap<getMin2(car_right[i].v+1,vmax)&&car_right[i].gap_back>=car_right[i].vob+diff_right) {
					car_right[i].isinsert = 1;
				} else {
					car_right[i].isinsert = 0;
				}
			} else {
				if (car_right[i].gap<getMin2(car_right[i].v+1,vmax) && d_pred_right >= car_right[i].gap+ car_right[i].v&&car_right[i].gap_back>=car_right[i].vob+diff_right) {
					car_right[i].isinsert = 1;
				} else {
					car_right[i].isinsert = 0;
				}
			}
		}
	}

	for (i = 0; i < veh_sum_left; i++) {
		if(car_left[i].gap_back<0||car_left[i].gap_other<0) {
			car_left[i].isinsert = 0;
		}
	}
	for (i = 0; i < veh_sum_right; i++) {
		if(car_right[i].gap_back<0||car_right[i].gap_other<0) {
			car_right[i].isinsert = 0;
		}
	}


	/*step3*/
	/*左车道换右车道*/
	for (i = 0; i < veh_sum_left; ) {
		if (car_left[i].isinsert == 1) {

			int leftNum = -1;
			if (car_left[i].x > car_right[0].x) {
				leftNum = 0;//插入位置在右车道最前面
			} else if (car_left[i].x < car_right[veh_sum_right - 1].x) {
				leftNum = veh_sum_right;//插入位置在右车道最后面
			} else {
				for (j = 0; j < veh_sum_right; j++) {
					if (car_left[i].x<car_right[j].x && car_left[i].x>car_right[j + 1].x) {
						leftNum = j + 1;
					}
				}
			}
			/*右车道情况*/
			for (k = veh_sum_right - 1; k >= leftNum; k--) {//前面车辆不要动，后面车辆往后移
				car_right[k + 1].x = car_right[k].x;
				car_right[k + 1].v = car_right[k].v;
				car_right[k + 1].isinsert = car_right[k].isinsert;
				car_right[k + 1].type = car_right[k].type;
				car_right[k + 1].tst = car_right[k].tst;
				car_right[k + 1].breaksta = car_right[k].breaksta;
			}
			car_right[leftNum].x = car_left[i].x;
			car_right[leftNum].v = car_left[i].v;
			car_right[leftNum].tst = car_left[i].tst;
			car_right[leftNum].type = car_left[i].type;
			car_right[leftNum].breaksta = car_left[i].breaksta;
			car_right[leftNum].isinsert = 0;
			veh_sum_right++;
			/*左车道情况*/
			for (j = i + 1; j <= veh_sum_left - 1; j++) {
				car_left[j - 1].x = car_left[j].x;
				car_left[j - 1].v = car_left[j].v;
				car_left[j - 1].tst = car_left[j].tst;
				car_left[j - 1].type = car_left[j].type;
				car_left[j - 1].breaksta = car_left[j].breaksta;
				car_left[j - 1].isinsert = car_left[j].isinsert;
			}
			veh_sum_left--;
			//若直接i++，则会直接跳过下一个需要监测的点，这里i--与i++抵消，下一循环判断下一辆车。
		} else {
			i = i + 1;
		}
	}



	/*右车道换左车道*/
	for (i = 0; i < veh_sum_right;) {//这里sum_right的值应该改变
		if (car_right[i].isinsert == 1) {
			int rightNum = -2;
			if (car_right[i].x > car_left[0].x) {
				rightNum = 0 ;//插入位置在左车道第一辆车前面
			} else if (car_right[i].x < car_left[veh_sum_left - 1].x) {
				rightNum = veh_sum_left;//插入位置在左车道最后一辆车后面
			} else {
				for (j = 0; j < veh_sum_left; j++) {
					if (car_right[i].x<car_left[j].x && car_right[i].x>car_left[j + 1].x) {//
						rightNum = j + 1;
					}
				}
			}
			/*左车道情况*/
			for (k = veh_sum_left - 1; k >= rightNum; k--) {
				car_left[k + 1].v = car_left[k].v;
				car_left[k + 1].x = car_left[k].x;
				car_left[k + 1].tst = car_left[k].tst;
				car_left[k + 1].type = car_left[k].type;
				car_left[k + 1].breaksta = car_left[k].breaksta;
				car_left[k + 1].isinsert = car_left[k].isinsert;
			}
			car_left[rightNum].x = car_right[i].x;
			car_left[rightNum].v = car_right[i].v;
			car_left[rightNum].tst = car_right[i].tst;
			car_left[rightNum].type = car_right[i].type;
			car_left[rightNum].breaksta = car_right[i].breaksta;
			car_left[rightNum].isinsert = 0;
			veh_sum_left++;
			/*右车道情况*/
			for (j = i + 1; j <= veh_sum_right - 1; j++) {
				car_right[j - 1].x = car_right[j].x;
				car_right[j - 1].v = car_right[j].v;
				car_right[j - 1].tst = car_right[j].tst;
				car_right[j - 1].type = car_right[j].type;
				car_right[j - 1].breaksta = car_right[j].breaksta;
				car_right[j - 1].isinsert = car_right[j].isinsert;
			}
			veh_sum_right--;
		} else {
			i++;
		}
	}


	//左车道统计间距
	for (i = 0; i < veh_sum_left; i++) {
		if(car_left[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_left[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("还有别的类型");
		}

		if (i == 0) {
			car_left[i].gap = car_left[veh_sum_left - 1].x - car_left[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}

		if (car_left[i].gap < 0) {
			printf("左2错误: 时步：%d 车辆总数：%d 编号：%d 间距：%d 车位置：%d 前车位置：%d 车辆长度：%d \n\n",time_step,veh_sum_left, i,car_left[i].gap, car_left[i].x,car_left[i-1].x,veh_length);
		}

	}

	//右车道再统计一次
	fortemp=0;//for循环改进地方
	for (i = 0; i < veh_sum_right; i++) {
		if(car_right[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_right[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("还有别的类型");
		}

		if (i == 0) {//最前面一辆车子
			car_right[i].gap = car_right[veh_sum_right - 1].x - car_right[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
			if (car_right[i].gap < 0) {
				printf("右2错误：时步；%d 车辆总数：%d 编号：%d 类型：%d 间距：%d 车辆位置：%d 前车位置；%d 车辆长度：%d\n\n",time_step,veh_sum_right,i,car_right[i].type,car_right[i].gap,car_right[i].x,car_right[i-1].x,veh_length);
			}
		}

	}

	/*step4*/
	for (i = 0; i < veh_sum_left; i++) {
		if(car_left[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else {
			vmax =vmax_long;
			veh_length=veh_length_long;
		}

		//保存上一时步的速度
		vleft_save[i]=car_left[i].v;

		//刹车概率
		if(car_left[i].v==0) {
			th_left=10000;
		} else {
			th_left=(double)car_left[i].gap/car_left[i].v;
		}

		//printf("%f",th_left);

		ts_left=(double)getMin2(car_left[i].v,h_con);

		if(i==0) {
			if(car_left[veh_sum_left-1].breaksta==1&&th_left<ts_left) {
				p_slow=pb;
			} else if(car_left[i].v==0) {
				p_slow=p0;
			} else {
				p_slow=pd;
			}
		} else {
			if(car_left[i-1].breaksta==1&&th_left<ts_left) {//修改
				p_slow=pb;
			} else if(car_left[i].v==0) {
				p_slow=p0;
			} else {
				p_slow=pd;
			}
		}

		//加速步
		if(i==0) {
			if((car_left[veh_sum_left-1].breaksta==0&&car_left[i].breaksta==0)||th_left>=ts_left) {//这里第0车前车是veh_sum_left-1
				car_left[i].v = getMin2(vmax, car_left[i].v + 1);//加速
			}
		} else {
			if((car_left[i-1].breaksta==0&&car_left[i].breaksta==0)||th_left>=ts_left) {
				car_left[i].v = getMin2(vmax, car_left[i].v + 1);//加速
			}
		}

		//减速步
		if(i==0) {
			vanti=getMin2(car_left[veh_sum_left-1].gap,car_left[veh_sum_left-1].v);
		} else {
			vanti=getMin2(car_left[i-1].gap,car_left[i-1].v);
		}


		int deff=car_left[i].gap+getMax2(vanti-gapsafe,0);

		car_left[i].v=getMin2(deff,car_left[i].v);

		//确定刹车灯状态
		if(car_left[i].v<vleft_save[i]) {
			car_left[i].breaksta=1;
		} else {
			car_left[i].breaksta=0;
		}

		//随机慢化
		if(((double)rand())/RAND_MAX<p_slow) {
			car_left[i].v=getMax2(car_left[i].v-1,0);
			if(p_slow==pb) {
				car_left[i].breaksta=1;
			}
		}

	}

	/*右车道NS模型*/
	for (i = 0; i < veh_sum_right; i++) {

		if(car_right[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else {
			vmax =vmax_long;
			veh_length=veh_length_long;
		}

		//保存上一时步的速度
		vright_save[i]=car_right[i].v;

		//刹车概率
		if(car_right[i].v==0) {
			th_right=10000;//为比较大的值
		} else {
			th_right=(double)car_right[i].gap/car_right[i].v;
		}
		ts_right=(double)getMin2(car_right[i].v,h_con);

		if(i==0) {
			if(car_right[veh_sum_right-1].breaksta==1&&th_right<ts_right) {
				p_slow=pb;
			} else if(car_right[i].v==0) {
				p_slow=p0;
			} else {
				p_slow=pd;
			}
		} else {
			if(car_right[i-1].breaksta==1&&th_right<ts_right) {
				p_slow=pb;
			} else if(car_right[i].v==0) {
				p_slow=p0;
			} else {
				p_slow=pd;
			}
		}

		//加速步
		if(i==0) {
			if((car_right[veh_sum_right-1].breaksta==0&&car_right[i].breaksta==0)||th_right>=ts_right) {
				car_right[i].v = getMin2(vmax, car_right[i].v + 1);//加速
			}
		} else {
			if((car_right[i-1].breaksta==0&&car_right[i].breaksta==0)||th_right>=ts_right) {
				car_right[i].v = getMin2(vmax, car_right[i].v + 1);//加速
			}
		}

		//减速步
		if(i==0) {
			vanti=getMin2(car_right[veh_sum_right-1].gap,car_right[veh_sum_right-1].v);
		} else {
			vanti=getMin2(car_right[i-1].gap,car_right[i-1].v); //这里直接为当前车道前车的速度，不为相邻车道前车速度
		}

		int deff=car_right[i].gap+getMax2(vanti-gapsafe,0);

		car_right[i].v=getMin2(deff,car_right[i].v);

		//确定刹车灯状态
		if(car_right[i].v<vright_save[i]) {
			car_right[i].breaksta=1;
		} else {
			car_right[i].breaksta=0;
		}

		//随机慢化
		if(((double)rand())/RAND_MAX<p_slow) {
			car_right[i].v=getMax2(car_right[i].v-1,0);
			if(p_slow==pb) {
				car_right[i].breaksta=1;
			}
		}
	}

	//位置更新
	for (i = 0; i < veh_sum_left; i++) {
		car_left[i].x = car_left[i].x + car_left[i].v;
	}

	for (i = 0; i < veh_sum_right; i++) {
		car_right[i].x = car_right[i].x + car_right[i].v;
	}


	for (i = 0; i < veh_sum_left; i++) {

		if(car_left[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_left[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("左车道当前时步：%d 还有别的类型\n",time_step);
		}

		if (i == 0) {//最前面一辆车子
			car_left[i].gap = car_left[veh_sum_left - 1].x - car_left[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if (car_left[i].gap < 0) {
			printf("左4错误 时步：%d 车辆总数：%d 编号：%d 位置：%d 类型：%d 间距：%d 前车位置：%d\n",time_step,veh_sum_left,i,car_left[i].x,car_left[i].type,car_left[i].gap,car_left[i-1].x);
		}

	}

	for (i = 0; i < veh_sum_right; i++) {

		if(car_right[i].type==0) {
			vmax=vmax_short;
			veh_length=veh_length_short;
		} else if(car_right[i].type==1) {
			vmax =vmax_long;
			veh_length=veh_length_long;
		} else {
			printf("右车道当前时步：%d 还有别的类型 编号：%d 类型：%d \n",time_step,i,car_right[i].type);
		}

		if (i == 0) {//最前面一辆车子
			car_right[i].gap = car_right[veh_sum_right - 1].x - car_right[i].x + road_long - veh_length;//周期条件下,第一辆车永远是0车
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if (car_right[i].gap < 0) {
			printf("右4错误 时步：%d 车辆总数:%d 编号：%d 位置：%d 类型：%d 间距：%d 前车位置：%d\n",time_step,veh_sum_right,i,car_right[i].x,car_right[i].type,car_right[i].gap,car_right[i-1].x);
		}


	}



	return 0;
}

//左道出车
void car_out(CARL* car_left, CARR* car_right) {

	while (car_left[0].x >= road_long) {

		int left_save_x = car_left[0].x;
		int left_save_v = car_left[0].v;
		int left_save_tst = car_left[0].tst;
		int left_save_type = car_left[0].type;
		int left_save_breaksta = car_left[0].breaksta;

		for (i = 1; i < veh_sum_left; i++) {
			car_left[i - 1].v = car_left[i].v;
			car_left[i - 1].x = car_left[i].x;
			car_left[i - 1].tst = car_left[i].tst;
			car_left[i - 1].type = car_left[i].type;
			car_left[i - 1].breaksta = car_left[i].breaksta;
		}

		car_left[veh_sum_left-1].v = left_save_v;
		car_left[veh_sum_left-1].tst = left_save_tst;
		car_left[veh_sum_left-1].x = left_save_x-road_long;//间距应该减去道路长度
		car_left[veh_sum_left-1].type = left_save_type;
		car_left[veh_sum_left-1].breaksta = left_save_breaksta;
	}

	while (car_right[0].x >= road_long) {

		int right_save_x = car_right[0].x;
		int right_save_tst = car_right[0].tst;
		int right_save_v = car_right[0].v;
		int right_save_type = car_right[0].type;
		int right_save_breaksta = car_right[0].breaksta;

		for (i = 1; i < veh_sum_right; i++) {
			car_right[i - 1].v = car_right[i].v;
			car_right[i - 1].x = car_right[i].x;
			car_right[i - 1].tst = car_right[i].tst;
			car_right[i - 1].type = car_right[i].type;
			car_right[i - 1].breaksta = car_right[i].breaksta;
		}

		car_right[veh_sum_right-1].v = right_save_v;
		car_right[veh_sum_right-1].tst = right_save_tst;
		car_right[veh_sum_right-1].x = right_save_x-road_long;//间距应该减去道路长度
		car_right[veh_sum_right-1].type = right_save_type;
		car_right[veh_sum_right-1].breaksta = right_save_breaksta;
	}

}

//主函数
int main() {
	CARL* car_left;
	car_left = (CARL*)malloc(sizeof(CARL) * 10200);
	CARR* car_right;
	car_right = (CARR*)malloc(sizeof(CARR) * 10200);

	srand((unsigned)time(NULL));		//随机数种子

	r_pro=0.0;//长车和短车的比例

	FILE* fp;
	if ((fp = fopen("双车道 周期 随机分布 相同速度 比例：0.0.txt", "a+")) == NULL) {//这里修改5  6  10  20
		printf("cannot open this file.\n");
		exit(0);
	}

	for (occupancy = 0.01; occupancy < 0.8; occupancy+=0.01) {

		printf("道路占有率为：%f 正在计算...\n", occupancy);

		for(sam=1; sam<=sample; sam++) {

			//道路初始化
			veh_sum_left = 0;
			veh_sum_right = 0;

			vel_sum_left = 0;
			vel_sum_right = 0;

			density_sum_left=0.0;
			density_sum_right=0.0;

			for (i = 0; i < 10200; i++) {
				car_left[i].x = 0;
				car_left[i].v = 0;
				car_left[i].gap = 0;
				car_left[i].tst = 0;
				car_left[i].position = 0;
				car_left[i].gap_back = -1;
				car_left[i].gap_other = -1;
				car_left[i].vob = 0;
				car_left[i].type=-1;
				car_left[i].breaksta=0;
				car_left[i].vpred=0;
				car_left[i].tarb=0;
				car_left[i].pred_gap=-1;
				car_right[i].x = 0;
				car_right[i].v = 0;
				car_right[i].gap = 0;
				car_right[i].tst = 0;
				car_right[i].position = 0;
				car_right[i].gap_back = -1;
				car_right[i].gap_other = -1;
				car_right[i].vob = 0;
				car_right[i].type=-1;
				car_right[i].vpred=0;
				car_right[i].tarb=0;
				car_right[i].pred_gap=-1;

			}

			//撒车
			roadRandomInitialize(car_left, car_right);

			//printf("左车道车辆总数为：%d 右车道车辆总数为：%d\n\n",veh_sum_left,veh_sum_right);//这里车辆撒车正确

			for (time_step = 0; time_step < total_timestep; time_step++) {

				//车速更新
				velocitySum(car_left, car_right);

				if (time_step >= total_timestep - count_timestep) {

					//左车道计算道路速度、密度
					double left_sumv=0.0;

					for (j = 0; j < veh_sum_left; j++) {

						left_sumv += car_left[j].v*1.5*3.6;  //一时步下，将所有车辆速度和累加，单位为km/h

					}

					vel_sum_left+=left_sumv/veh_sum_left;//一时步平均速度

					density_sum_left+=(double)veh_sum_left/15; //一时步下的平均密度道路长度为15公里 

					//printf("当前时步：%d 车辆总数：%d 平均速度：%f 平均密度：%f\n",time_step,veh_sum_left,vel_sum_left,density_sum_left);



					//右车道计算道路速度、密度
					double right_sumv=0.0;

					for (j = 0; j < veh_sum_right; j++) {

						right_sumv+= car_right[j].v*1.5*3.6;

					}

					vel_sum_right += right_sumv / veh_sum_right;//当前时步平均速度

					density_sum_right += (double)veh_sum_right / 15; //当前时步下的平均密度

					//printf("当前时步：%d 车辆总数：%d 平均速度：%f 平均密度：%f\n",time_step,veh_sum_right,vel_sum_right,density_sum_right);

				}

				//出车系统
				car_out(car_left,car_right);
			}

			double Aver_v_left = vel_sum_left / (count_timestep);

			double Aver_den_left = density_sum_left / ( count_timestep);

			double Aver_flux_left = Aver_v_left * Aver_den_left;

			double Aver_v_right = vel_sum_right / (count_timestep);

			double Aver_den_right = density_sum_right / ( count_timestep);

			double Aver_flux_right = Aver_v_right * Aver_den_right;

			fprintf(fp, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \n", occupancy, Aver_v_left, Aver_den_left,Aver_flux_left, Aver_v_right,Aver_den_right, Aver_flux_right);

			printf("当前占有率为：%f ，样本数为：%d 左车道平均流量为：%f 右车道平均流量为：%f\n\n",occupancy,sam,Aver_flux_left,Aver_flux_right) ;

		}


	}
	
	fclose(fp);

	return 0;
}

