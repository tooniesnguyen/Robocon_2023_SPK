# Cách điều khiển động cơ DC Motor
- Để hiểu rõ sâu hơn về điều khiển động cơ mọi người có thể tham khảo bài giảng của Thầy Chương [1]
<br><img src="../Image/Diagrama%20BTS7960.png" width="1000px" height="500px"> </br> 
## Yêu cầu phần cứng:
- [Mạch nạp STM32](https://hshop.vn/products/mach-nap-st-link-v2)
- [STM32F103C8T6](https://hshop.vn/products/kit-ra-chon-stm32f103c8t6)
- [BTS7960 43A](https://hshop.vn/products/mach-dieu-khien-dong-co-dc-bts796043a-1-dong-co)
- Nguồn tổ ông 12V
- [Động Cơ DC Servo JGB37-520 DC Geared Motor](https://hshop.vn/products/dong-co-dc-servo-giamtoc-ga37-v1)

## Setup STM32F103C8T6
- `SYS` -> `Serial Wire`
- Cấu hình chân đọc encoder: `PB1`-> `GPIO_EXTI1`| `PB10` -> `GPIO_Input` [3] 
- `GPIO` : `Pull-up` cả hai
- `NVIC` -> `EXTI line1 interupt` -> Enabled
- `RCC` -> `HSE` -> `Crystal/Ceramic Resonator` [2]
- `Clock Configuration` (Chọn từ dưới cùng qua phải) -> `HSE` -> `PLLMul`: X9 -> `PLLCLK` -> `HCLK`:72 -> `APB1 Prescaler`: `/2` [2] 
#### Cấu hình timer để xuất xung PWM và PID
#### TIM2 cho PWM: 
- `Clock Source`: Internal Clock
- `Channel3`: PWM Generation CH3 (PA2)
- `Channel4`: PWM Generation CH4 (PA3)
-  Tiếp theo vào `Parameter Settings` -> `Perscaler`: 72-1 -> `Counter Period`: 1000-1 (Volt đạt tối đa khi PWM là 1000 giống 255 trong pwm arduino) [2]
#### TIM2 cho PID:
- Chọn `Internal Clock` -> `Parameter Settings` -> `Perscaler`: 72-1 -> `Counter Period`: 1000-1 (1ms lấy mẫu tính PID 1 lần) ->`NVIC Settings`: Enabled 


## Bản kết nối phần cứng:
**Ghi chú**:
| GPIO | Chức năng                             |
|------|---------------------------------------|
| PA2  | Xuất PWM cho động cơ quay 1 chiều     |
| PA3  | Xuất PWM cho động cơ quay ngược chiều |
| PB1  | Đọc ngắt Encoder [2]                  |
| PB10 | Đọc ngắt Encoder [2]                  |
#### Thứ tự nối chân
- Vd: PA2 -> RPWM, M+ -> M1, 3.3V + R_En + L_En + VCC thành một, ...

| Tổ ong 12V | STM32 |      BTS      | Động cơ DC |
|:----------:|:-----:|:-------------:|:----------:|
|            |       |       M+      |     M1     |
|            |       |       M-      |     M2     |
|            |  3.3V | R_EN,L_EN,VCC |     VCC    |
|            |  GND  |      GND      |     GND    |
|            |  PA2  |      RPWM     |            |
|            |  PA3  |      LPWM     |            |
|            |  PB1  |               |     C1     |
|            |  PB10 |               |     C2     |
|     V+     |       |       B+      |            |
|     V-     |       |       B-      |            |

## Code:
- Thêm thư viện stdlib để tính
```
/* USER CODE BEGIN Includes */
#include<stdlib.h>
/* USER CODE END Includes */
```
- Khai báo các biến cần thiết với 1 của velocity, 2 của position
- Trong phần hướng dẫn này chỉ dùng bộ điều khiển PI nên khâu D không cần thiết
```
/* USER CODE BEGIN PV */

int  count1, precount1, dir , pwm,  v_target = 0;
double v1, pre1, e1, ei1, ed1, u1, v1Filt, v1Prev, gain, pos;
// e1:  lỗi khâu P; ei1: lỗi khâu I
double kp1 = 0.4,ki1 =0.001; // Thông số kp ki để hệ thống ổn định

int p_target;
double e2, ei2, ed2, pre_e2, u2, kp2 , ki2 , kd2 ;
/* USER CODE END PV */
```
- Đọc ngắt encoder phục vụ cho tính toán PID [3]

```
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1){
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == 0) {count1++;}
		else {count1--;}
	}
}
```
- Tạo hàm xuất xung  PWM và điều chỉnh hướng quay (dir) cho động cơ:
```
void ControlMotor(int ChannelA, int ChannelB){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ChannelA);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ChannelB);
}

void driveSpeed(int dir , int pwmVal){
	if (dir == -1){
		ControlMotor(pwmVal,0);
	}
	else if (dir == 1){
		ControlMotor(0,pwmVal);
	}
	else{
		ControlMotor(0,0);
	}
}
```

- Tính toán PID vị trí (position): Ví dụ muốn động cơ quay 3 vòng ngừng thì PID vị trí giúp quay được 3 vòng một cách chính xác và ổn định không bị vọt lố hay đáp ứng chậm
- Với một hệ thống PID, động cơ DC được điều khiển bằng cách đo vị trí hiện tại của nó và so sánh với vị trí mong muốn, sau đó tính toán điện áp cần thiết để đưa động cơ DC đến vị trí đó.

	- Proportional (P): Đây là thành phần chính của hệ thống PID, nó xác định điện áp cần thiết để giảm khoảng cách giữa vị trí hiện tại và vị trí mong muốn. Thành phần này sử dụng một hệ số tỷ lệ để tính toán điện áp cần thiết.

	- Integral (I): Thành phần I giúp giảm độ sai lệch dài hạn giữa vị trí hiện tại và vị trí mong muốn. Thành phần này tính toán điện áp dựa trên một giá trị tích lũy của độ sai lệch.

	- Derivative (D): Thành phần D giúp giảm tốc độ của động cơ DC khi đến gần vị trí mong muốn, tránh hiện tượng quá bật hoặc giật khi đến gần vị trí mong muốn. Thành phần này tính toán điện áp dựa trên sự thay đổi của độ sai lệch.

- Sau khi tính toán ra u (output) sẽ quyết định pwm và hướng (dấu +- quyết định dir (hướng để động cơ quay tiếp hay hãm lại )) cho động cơ ổn định và chính xác theo mong muốn 
```
void posControlPID(){
	e2 = p_target - count1;
	ei2 += e2*0.001;
	ed2 = (e2-pre_e2)/0.001;
	if (ei2> 1000){
		ei2  =1000;
	}else if(ei2<-1000){
		ei2 = -1000;
	}
	u2 = e2*kp2+ei2*ki2+ed2*kd2;
	if (u2>0)dir=-1;
	else if(u2<0)dir = 1;
	else dir = 0;
	pwm = abs(u2);
	if(pwm>200){pwm = 200;}
	else if ((pwm<50)&&(e2!=0)){
		pwm = 50;

	}
}
```
- Tính toán PID vận tốc (velocity): tương tự PID vị trí nhằm mục đích cho động cơ quay vận tốc mong muốn và đáp ứng nhanh [4]
```
void calculatePIDSpeed(){
	e1 = v_target - v1; // tinh toan loi ty le
	ei1 += e1 * 0.001; // tinh toan loi tich phan
	ed1 = (e1 - pre1) / 0.001;
	u1 = kp1 * e1 + ki1 * ei1; //Tinh tong bo dieu khien
	pre1 = e1;
	if(u1 < 0) dir = 1; //bien doi chiue vong quay
	else dir = -1;
	pwm = abs(u1);

	if(pwm > 1000) pwm = 1000;//Bao hoa xung cap
	// Vì trong setup phía trên chúng ta cho pwm tối đa 1000 mà u có thể lơn hơn 1000 nên bão hòa
}
```
- Thực hiện trong `while` của hàm `main`:
- Chạy 1 trong 2 cái:
	- Nếu chạy vị trí thì `calculatePIDSpeed();`
	- Nếu chạy vận tốc thì `posControlPID();`

```
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  
//	calculatePIDSpeed();
	posControlPID();
	driveSpeed(dir,pwm);

    /* USER CODE END WHILE */
```

## Tài liệu tham khảo:
- [1] [Điều khiển động cơ Thầy Võ Lâm Chương](https://www.youtube.com/watch?v=VUqa3zvIrp8&list=PLpC3GniHRC0NSpHS_Y8AzRGS1mAb4zcHJ&ab_channel=Ph%E1%BA%A1mMinhTu%E1%BA%A5n)
- [2] [Cấu hình clock,timer,pwm](https://youtu.be/OwlfFp8fPN0)
- [3] [Training Encoder của Duy](https://youtu.be/otyEy5TXrLs)
- [4] [DC motor PID speed](https://youtu.be/HRaZLCBFVDE)
