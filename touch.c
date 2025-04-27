#include "stm32f4xx_hal.h"       // STM32F4 시리즈용 HAL 드라이버 헤더 파일
#include "touch.h"               // 터치 기능 관련 헤더 파일
#include "graphics.h"            // 그래픽 처리용 헤더 파일
#include "calibrate.h"           // 터치 보정(calibration) 관련 헤더 파일

// 터치 컨트롤러로부터 좌표를 읽기 위한 명령어
#define COMMAND_READ_X             			0xD0
#define COMMAND_READ_Y             			0x90

// 터치 좌표 읽을 때 사용할 샘플 수
#define MW_HAL_TOUCH_READ_POINTS_COUNT		10U

// 터치 컨트롤러 칩 선택 (CS 핀 활성화)
#define CS_ON								(HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET))
// 터치 컨트롤러 칩 선택 해제 (CS 핀 비활성화)
#define CS_OFF 								(HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET))

// 터치 인터럽트 핀 설정
#define TOUCH_IRQ_PORT						GPIOA
#define TOUCH_IRQ_PIN						GPIO_PIN_3

extern SPI_HandleTypeDef hspi1;              // SPI1 핸들 선언 (터치 컨트롤러 통신용)

static MATRIX matrix;                       // 터치 보정에 사용될 변환 행렬

// 내부 함수 선언
static void DrawCross(int16_t x, int16_t y, int16_t length); // 십자 표시를 그리는 함수
static uint8_t SpiTransfer(uint8_t byte);                    // SPI 전송 및 수신 함수
static bool GetPointRaw(uint16_t* x, uint16_t* y);            // 터치 원시(raw) 좌표 읽기 함수

// SPI를 통해 1바이트 전송 후 수신 결과를 반환하는 함수
static uint8_t SpiTransfer(uint8_t byte)
{
	uint8_t result;

	(void)HAL_SPI_TransmitReceive(&hspi1, &byte, &result, 1U, 1000U); // SPI 전송 및 수신

	return (result); // 수신된 데이터 반환
}

// 십자 표시를 화면에 그리는 함수
static void DrawCross(int16_t x, int16_t y, int16_t length)
{
	GraphicsClear(WHITE);                            // 화면을 흰색으로 초기화
	GraphicsHline(x - length / 2, x + length / 2, y, BLACK); // 가로 선 그리기
	GraphicsVline(x, y - length / 2, y + length / 2, BLACK); // 세로 선 그리기
	GraphicsStandardString(50, 150, "    Calibration !!!", BLACK); // 안내 문구 표시
}

// 터치 스크린으로부터 원시 좌표를 읽어오는 함수
static bool GetPointRaw(uint16_t* x, uint16_t* y)
{
	uint8_t i;
 	bool sorted;
 	uint16_t swap_value;
	uint16_t x_raw;
	uint16_t y_raw;
	uint16_t databuffer[2][MW_HAL_TOUCH_READ_POINTS_COUNT]; // x와 y를 각각 저장하는 버퍼
	uint8_t touch_count;

	// 터치가 감지되지 않으면 종료
	if (!TouchIsTouched())
	{
		return false;
	}

	// 터치가 유지되는 동안 여러 번 좌표 읽기
	CS_ON; // 터치 컨트롤러 선택
	touch_count = 0U;
	do
	{
		// x좌표 읽기
		SpiTransfer(COMMAND_READ_X);
		x_raw = (uint16_t)SpiTransfer(0U) << 8;
		x_raw |= (uint16_t)SpiTransfer(0U);
		x_raw >>= 3; // 필요한 만큼 비트 쉬프트

		// y좌표 읽기
		SpiTransfer(COMMAND_READ_Y);
		y_raw = (uint16_t)SpiTransfer(0U) << 8;
		y_raw |= (uint16_t)SpiTransfer(0U);
		y_raw >>= 3; // 필요한 만큼 비트 쉬프트

		// 읽은 값을 버퍼에 저장
		databuffer[0][touch_count] = x_raw;
		databuffer[1][touch_count] = y_raw;
		touch_count++;
	}
	while (TouchIsTouched() == true && touch_count < MW_HAL_TOUCH_READ_POINTS_COUNT);
	CS_OFF; // 터치 컨트롤러 선택 해제

	// 읽은 개수가 부족하면 무효
	if (touch_count != MW_HAL_TOUCH_READ_POINTS_COUNT)
	{
		return false;
	}

	// x값 정렬
	do
	{
		sorted = true;
		for (i = 0U; i < touch_count - 1U; i++)
		{
			if(databuffer[0][i] > databuffer[0][i + 1U])
			{
				swap_value = databuffer[0][i + 1U];
				databuffer[0][i + 1U] = databuffer[0][i];
				databuffer[0][i] = swap_value;
				sorted = false;
			}
		}
	}
	while (!sorted);

	// y값 정렬
	do
	{
		sorted = true;
		for (i = 0U; i < touch_count - 1U; i++)
		{
			if (databuffer[1][i] > databuffer[1][i + 1U])
			{
				swap_value = databuffer[1][i + 1U];
				databuffer[1][i + 1U] = databuffer[1][i];
				databuffer[1][i] = swap_value;
				sorted = false;
			}
		}
	}
	while (!sorted);

	// 중간값 2개를 평균하여 노이즈 제거
	*x = (databuffer[0][4] + databuffer[0][5]) / 2U;
	*y = (databuffer[1][4] + databuffer[1][5]) / 2U;

	return true;
}

// 터치가 눌렸는지 확인하는 함수
bool TouchIsTouched(void)
{
	GPIO_PinState pin_state = HAL_GPIO_ReadPin(TOUCH_IRQ_PORT, TOUCH_IRQ_PIN); // 터치 인터럽트 핀 상태 읽기
	return pin_state == GPIO_PIN_RESET; // LOW이면 터치됨
}

// 보정된 터치 좌표를 반환하는 함수
bool TouchGetCalibratedPoint(int16_t* x, int16_t* y)
{
	POINT_T raw_point;
	POINT_T display_point;
	uint16_t raw_x;
	uint16_t raw_y;

	// 원시 좌표 읽기
	if (GetPointRaw(&raw_x, &raw_y) == false)
	{
		return false;
	}

	raw_point.x = (INT_32)raw_x;
	raw_point.y = (INT_32)raw_y;

	// 보정 행렬을 적용하여 디스플레이 좌표 변환
	(void)getDisplayPoint(&display_point, &raw_point, &matrix);

	// 디스플레이 범위를 벗어나지 않도록 제한
	if (display_point.x > 239)
	{
		display_point.x = 239;
	}
	if (display_point.y > 319)
	{
		display_point.y = 319;
	}

	if (display_point.x < 0)
	{
		display_point.x = 0;
	}
	if (display_point.y < 0)
	{
		display_point.y = 0;
	}

	*x = (int16_t)display_point.x;
	*y = (int16_t)display_point.y;

	return true;
}

// 터치스크린 보정(calibration) 절차를 수행하는 함수
void TouchCalibrate(void)
{
	uint16_t x;
	uint16_t y;
	POINT_T raw_points[3];                  // 원시 좌표 저장용
	POINT_T display_points[3] = {{40, 40}, {200, 40}, {200, 280}}; // 디스플레이상의 목표 좌표

    /* 첫 번째 점 */
	DrawCross(40, 40, 40);                // 십자 표시
	while (TouchIsTouched() == false) {}   // 터치될 때까지 대기
	while (GetPointRaw(&x, &y) == false) {} // 원시 좌표 읽기
	raw_points[0].x = (INT_32)x;
	raw_points[0].y = (INT_32)y;
	while (TouchIsTouched() == true) {}    // 터치 해제될 때까지 대기

    /* 두 번째 점 */
	DrawCross(200, 40, 40);
	while (TouchIsTouched() == false) {}
	while (GetPointRaw(&x, &y) == false) {}
	raw_points[1].x = (INT_32)x;
	raw_points[1].y = (INT_32)y;
	while (TouchIsTouched() == true) {}

    /* 세 번째 점 */
	DrawCross(200, 280, 40);
	while (TouchIsTouched() == false) {}
	while (GetPointRaw(&x, &y) == false) {}
	raw_points[2].x = (INT_32)x;
	raw_points[2].y = (INT_32)y;
	while (TouchIsTouched() == true) {}

	// 보정 행렬 생성
	(void)setCalibrationMatrix(display_points, raw_points, &matrix);
}
