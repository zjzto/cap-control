 /* Infinite loop */
  	SystemClock_Init(HSE_Clock); 
	/* 初始化GPIO */
	
        Delay(0x00fff);
     
	/*GPIOB5为a相电压同步信号输入,GPIOB4为b相电压同步信号输入,GPIOB3为c相电压同步信号输入,*/
        
        Relay_Gpio_Init();
	GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_IT);//GPIOB5,输入，弱上拉输入，A 相同步
	GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_IT);//GPIOB4,输入，弱上拉输入，B 相同步
	GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_IN_FL_IT);//GPIOB3,输入，弱上拉输入，C 相同步
        GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_IN_FL_IT);//GPIOE5,输入，弱上拉输入，D 相同步12-11

        GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_FL_IT);//PIN60  FEEDBACK_B
        GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_IT);//59     FEEDBACK_A
        GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_IN_FL_IT);//62     FEEDBACK_C 12-11
        GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_IN_FL_IT);//61     FEEDBACK_D 12-11
 
        /*RS485 read and write*/
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_SLOW);//RS485通讯使能端 PA3
	GPIO_Init(GPIOA, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);//RS485通讯输入端需配置IO:PA4
	GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_SLOW);//RS485通讯输 出端需配置IO:PA5
        
        LED_Key_Init();
        LCD_Init();
        ADC_Init();
        LCD_Clear();
        Uart1_Init_User();
	UART1_ReceiveData8();
        Tim1_Init();
        Tim2_Init();
        Tim4_Init();
        EXTI_DeInit();
       	g_VoltageModifyA=(uint16_t)(ReadEEPROM(VA_OFFSET+1));
	g_VoltageModifyA|=(((uint16_t)ReadEEPROM(VA_OFFSET))<<8);
	if((g_VoltageModifyA>=500)&&(g_VoltageModifyA<=650))
		{g_VoltageModifyA=g_VoltageModifyA;}
	else {g_VoltageModifyA=573;}	g_VoltageModifyB=(uint16_t)(ReadEEPROM(VB_OFFSET+1));
	g_VoltageModifyB|=(((uint16_t)ReadEEPROM(VB_OFFSET))<<8);
	if((g_VoltageModifyB>=500)&&(g_VoltageModifyB<=650))
		{g_VoltageModifyB=g_VoltageModifyB;}
	else {g_VoltageModifyB=580;}
	g_VoltageModifyC=(uint16_t)(ReadEEPROM(VC_OFFSET+1));
	g_VoltageModifyC|=(((uint16_t)ReadEEPROM(VC_OFFSET))<<8);
	if((g_VoltageModifyC>=500)&&(g_VoltageModifyC<=650))
		{g_VoltageModifyC=g_VoltageModifyC;}
	else {g_VoltageModifyC=580;}
	g_OverVoltage=(uint16_t)(ReadEEPROM(OVER_VOLTAGE+1));
	g_OverVoltage|=(((uint16_t)ReadEEPROM(OVER_VOLTAGE))<<8);
	if((g_OverVoltage>=240)&&(g_OverVoltage<=260))
		{g_OverVoltage=g_OverVoltage;}
	else {g_OverVoltage=240;}
	g_LowerVoltage=(uint8_t)(ReadEEPROM(LOWR_VOLTAGE));
	if((g_LowerVoltage>=175)&&(g_LowerVoltage<=195))
		{g_LowerVoltage=g_LowerVoltage;}
	else {g_LowerVoltage=175;}
	g_OverTemperature=(uint8_t)(ReadEEPROM(OVER_TEMPERA));
	if((g_OverTemperature>=55)&&(g_OverTemperature<=70))
		{g_OverTemperature=g_OverTemperature;}
	else {g_OverTemperature=55;}
	g_CounterPointA1=(uint16_t)(ReadEEPROM(A_REL_COUNTERH+1));
	g_CounterPointA1|=(((uint16_t)ReadEEPROM(A_REL_COUNTERH))<<8);
	if((g_CounterPointA1>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointA1<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointA1=g_CounterPointA1;}
	else {g_CounterPointA1= (u16)((u32)SAMPLING_POINTS*70/100);}	
	g_CounterPointB1=(uint16_t)(ReadEEPROM(B_REL_COUNTERH+1));
	g_CounterPointB1|=(((uint16_t)ReadEEPROM(B_REL_COUNTERH))<<8);
	if((g_CounterPointB1>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointB1<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointB1=g_CounterPointB1;}
	else {g_CounterPointB1= (u16)((u32)SAMPLING_POINTS*70/100);}	
	g_CounterPointC1=(uint16_t)(ReadEEPROM(C_REL_COUNTERH+1));
	g_CounterPointC1|=(((uint16_t)ReadEEPROM(C_REL_COUNTERH))<<8);
	if((g_CounterPointC1>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointC1<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointC1=g_CounterPointC1;}
	else {g_CounterPointC1= (u16)((u32)SAMPLING_POINTS*70/100);}	

        g_CounterPointD1=(uint16_t)(ReadEEPROM(D_REL_COUNTERH+1));
	g_CounterPointD1|=(((uint16_t)ReadEEPROM(D_REL_COUNTERH))<<8);
	if((g_CounterPointD1>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointD1<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointD1=g_CounterPointD1;}
	else {g_CounterPointD1= (u16)((u32)SAMPLING_POINTS*70/100);}	
		
	g_IPAdress=(uint8_t)(ReadEEPROM(ADDRESS));
	if(g_IPAdress<=24)
		{g_IPAdress=g_IPAdress;}
	else {g_IPAdress=2;}	

	g_EquipmentNumber=(uint8_t)(ReadEEPROM(CAP_NUM));
	if((g_EquipmentNumber>=1)&&(g_EquipmentNumber<=25))
		{g_EquipmentNumber=g_EquipmentNumber;}
	else {g_EquipmentNumber=2;}

	g_CTPhaseA=(uint16_t)(ReadEEPROM(CT+1));
	g_CTPhaseA|=(((uint16_t)ReadEEPROM(CT))<<8);
	if(g_CTPhaseA<=800)
		{g_CTPhaseA=g_CTPhaseA;}
	else {g_CTPhaseA=200;}

	g_TotalCycleTimeNum=(uint8_t)(ReadEEPROM(PERIOD));
	if((g_TotalCycleTimeNum>3)&&(g_TotalCycleTimeNum<=200))
		{g_TotalCycleTimeNum=g_TotalCycleTimeNum;}
	else {g_TotalCycleTimeNum=4;}
	
	g_MasterSlave=(uint8_t)(ReadEEPROM(MAS_SLA));
	if(g_MasterSlave<=2)
		{g_MasterSlave=g_MasterSlave;}
	else {g_MasterSlave=0;}
	

	g_CounterPointA2=(uint16_t)(ReadEEPROM(A_REL_COUNTERL+1));
	g_CounterPointA2|=(((uint16_t)ReadEEPROM(A_REL_COUNTERL))<<8);
	if((g_CounterPointA2>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointA2<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointA2=g_CounterPointA2;}
	else {g_CounterPointA2= (u16)((u32)SAMPLING_POINTS*70/100);}	

	g_CounterPointB2=(uint16_t)(ReadEEPROM(B_REL_COUNTERL+1));
	g_CounterPointB2|=(((uint16_t)ReadEEPROM(B_REL_COUNTERL))<<8);
	if((g_CounterPointB2>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointB2<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointB2=g_CounterPointB2;}
	else {g_CounterPointB2= (u16)((u32)SAMPLING_POINTS*70/100);}	

	g_CounterPointC2=(uint16_t)(ReadEEPROM(C_REL_COUNTERL+1));
	g_CounterPointC2|=(((uint16_t)ReadEEPROM(C_REL_COUNTERL))<<8);
	if((g_CounterPointC2>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointC2<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointC2=g_CounterPointC2;}
	else {g_CounterPointC2= (u16)((u32)SAMPLING_POINTS*70/100);}	
        
        g_CounterPointD2=(uint16_t)(ReadEEPROM(D_REL_COUNTERL+1));
	g_CounterPointD2|=(((uint16_t)ReadEEPROM(D_REL_COUNTERL))<<8);
	if((g_CounterPointD2>= (u16)((u32)SAMPLING_POINTS*50/100))&&(g_CounterPointD2<= (u16)((u32)SAMPLING_POINTS*90/100)))
		{g_CounterPointD2=g_CounterPointD2;}
	else {g_CounterPointD2= (u16)((u32)SAMPLING_POINTS*70/100);}
	g_PFTou=(uint16_t)(ReadEEPROM(PF_LLIM+1));
	g_PFTou|=(((uint16_t)ReadEEPROM(PF_LLIM))<<8);
	if((g_PFTou>=920)&&(g_PFTou<970))
		{g_PFTou=g_PFTou;}
	else {g_PFTou=920;}	

	g_PFQie=(uint16_t)(ReadEEPROM(PF_HLIM+1));
	g_PFQie|=(((uint16_t)ReadEEPROM(PF_HLIM))<<8);
	if((g_PFQie>=950)&&(g_PFQie<=990))
		{g_PFQie=g_PFQie;}
	else {g_PFQie=960;}	


	g_ReactiveQie=(uint16_t)(ReadEEPROM(REACT_L));
	if((g_ReactiveQie>0)&&(g_ReactiveQie<=50))
		{g_ReactiveQie=g_ReactiveQie;}
	else {g_ReactiveQie=30;}	

	g_SepCom=(uint8_t)(ReadEEPROM(COM_SEP));
	if(g_SepCom<=1)
		{g_SepCom=g_SepCom;}
	else {g_SepCom=0;}

        A_Counter=ReadEEPROM(A_RELAY_COUNTER+1);     //A相投切次数计数存储
        A_Counter|=(ReadEEPROM(A_RELAY_COUNTER)<<8);
		
        if(g_CTPhaseA>800) {g_CTPhaseA=10;}
	if(g_OverVoltage>260) {g_OverVoltage=260;}
	if(g_OverVoltage<240) {g_OverVoltage=240;}
	if(g_LowerVoltage>195) {g_LowerVoltage=195;}
	if(g_LowerVoltage<175) {g_LowerVoltage=175;}
	if(g_OverTemperature>70) {g_OverTemperature=70;}
	if(g_OverTemperature<55) {g_OverTemperature=55;}
	
        Reset_State=1;	
	if(g_IPAdress>25) 
        {
          g_IPAdress=1;
        }
        if((g_MasterSlave==0)||(g_MasterSlave==1)) //设为主机，从机模式直接进入静默地址模式
        {
          Add_Alloc_Done=1;
          UART1->CR2 |= (uint8_t)(UART1_CR2_RWU); //使能RWU
          UART1->CR1 |= 0x08;                     //WAKE被地址标记唤醒
          if((g_IPAdress>=1)&&(g_IPAdress<=14))   { UART1_SetAddress(g_IPAdress);}//IP地址写入ADD
          if((g_IPAdress>14)&&(g_IPAdress<=25))   { UART1_SetAddress(0x0f);}//IP地址写入ADD
        }
        else 
          TIM4_Cmd(ENABLE);       //侯址模式并打开TIM4进行等待4.45min
	if(g_EquipmentNumber>25) 
          g_EquipmentNumber=1;
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);//GPIOB中断上升沿有效
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);//GPIOD中断上升沿有效
        EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOE, EXTI_SENSITIVITY_RISE_ONLY);//GPIOE中断上升沿有效
    	DisableCap();
  	CS5463_Init();
	
	if(g_MasterSlave!=1)
          MAS_LED_OFF;
       
	if(g_MasterSlave==1)
          MAS_LED_ON;
        RUN_LED_OFF;
        ERR_LED_OFF;
        UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);//使能接收中断//9-19
        UART1->CR2 |= (uint8_t)(UART1_CR2_REN); //使能接收
        UART1->CR2 &= (uint8_t)(~UART1_CR2_TEN); //发送被 禁止
        Delay(0x0ff);

  while (1)
  {
    	
  	/*检查手动和自动，g_AutoManual=1时为手动,开关左偏*/
	if(!GPIO_ReadInputPin(KEY_PORTE, KEY_7))
        {	
          if(!GPIO_ReadInputPin(KEY_PORTE, KEY_7))
            g_AutoManual=1;
        }		
	else		
        {g_AutoManual=0;}
	if((g_MasterSlave==1)&&(g_IPAdress!=1)) {g_AutoManual=1;}
	if((g_MasterSlave!=1)&&(g_IPAdress==1)) {g_AutoManual=1;}
	
        /* 手动运行，开关关闭状态下，禁止自动运行*/
	if(g_AutoManual==1) 
        {
		g_CloseState=0;//进入运行状态
		u8 g_DisplayNumber=0;//手动第一页选择变量
		u8 g_ChooseNumber1=0;//手动第二页选择变量
		u8 g_ChooseNumber2=0;// 手动参数设置页
		u8 g_ChooseNumber3=0;//手动延时设置页变量
                u8 g_ChooseNumber3_1=0;//手动地址设置变量
                u8 g_ChooseNumber3_2=0;//手动主从设置变量
                u8 g_ChooseNumber3_3=0;//CT变比设置变量
                u8 g_ChooseNumber3_4=0;//自动校准过零
                u8 g_ChooseNumber3_5=0;//电容投切大循环时间设置        
                u8 g_ChooseNumber3_6=0;// 功率因数和无功阀值设置       
		u8 g_ChooseNumber4=0;//手动投切页面
		u8 g_ChooseNumber5=2;//手动投切从机页面
                u16 g_BackCounter1=0;//显示计时返回变量
                uint8_t PhaseABC=0;
		g_OnNum1=0;
		g_OnNum2=0;
		g_OnNum3=0;
		g_OffNum1=0;
		g_OffNum2=0;
		g_OffNum3=0;
		g_TotalOnNumber1=0;
		g_TotalOnNumber2=0;
		g_TotalOnNumber3=0;
                GPIOA->ODR &= (uint8_t)(~GPIO_PIN_3);//SP485E 发送禁止
		UART1_ITConfig(UART1_IT_RXNE_OR, DISABLE);//禁止接收中断
		UART1->CR2 &= (uint8_t)(~UART1_CR2_REN);//禁止接收
		UART1->CR2 &= (uint8_t)(~UART1_CR2_TEN); //发送被 禁止
		DisableCap();
                EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
		enableInterrupts();
                EnableKey();
		Display1Page();
		LCD_Display(Bmp_XuanZe, 0xb4, 0x13,0x00, 16);
		LCD_Display(Bmp_XuanZe+16, 0xb5, 0x13,0x00, 16);
		while(1) {
                 Delay(0x00006fff);     
                 g_BackCounter1=g_BackCounter1+1;
                 
                
                if((g_Key1StateCheck==1)&&(g_DisplayNumber==0)) {	
                  g_BackCounter1=0; 
                  ClearDisplay(0xb4, 0xb5,0x13,0x00);
                  XuanZeDisplay(0xb4, 0xb5,0x16,0x08);
                  g_Key1StateCheck=0;
                  g_DisplayNumber=1;
                                                                }
                if((g_Key1StateCheck==1)&&(g_DisplayNumber==1)) {	
                  g_BackCounter1=0;
                  ClearDisplay(0xb4, 0xb5,0x16,0x08);
                  XuanZeDisplay(0xb6, 0xb7,0x13,0x00);
                  g_Key1StateCheck=0;
                  g_DisplayNumber=2;
                                                                }
                if((g_Key1StateCheck==1)&&(g_DisplayNumber==2)) {	
                  g_BackCounter1=0;
                  ClearDisplay(0xb6, 0xb7,0x13,0x00);
                  XuanZeDisplay(0xb6, 0xb7,0x16,0x08);
                  g_DisplayNumber=3;
                  g_Key1StateCheck=0;
                                                                }	
                if((g_Key1StateCheck==1)&&(g_DisplayNumber==3)) {	
                  g_BackCounter1=0;
                  ClearDisplay(0xb6, 0xb7,0x16,0x08);
                  XuanZeDisplay(0xb4, 0xb5,0x13,0x00);
                  g_Key1StateCheck=0;
                  g_DisplayNumber=0;
                                                                }	
                if((g_Key2StateCheck==1)&&(g_DisplayNumber==0)) {	
                  g_BackCounter1=0;
                   Display2_1Page();
                    LCD_Display(Bmp_XuanZe, 0xb0, 0x12,0x0d, 16);
                    LCD_Display(Bmp_XuanZe+16, 0xb1, 0x12,0x0d, 16);
                    g_Key2StateCheck=0;
                    while(1) 
                    { 	
                      Delay(0x00006fff);     
                      g_BackCounter1=g_BackCounter1+1;
                      if(g_BackCounter1>100)
                        {    g_BackCounter1=0;
                            g_ChooseNumber1=0;
                            break;
                        }
                                                                            
                    if((g_Key1StateCheck==1)&&(g_ChooseNumber1==0)) {               
                      g_BackCounter1=0;
                      ClearDisplay(0xb0, 0xb1,0x12,0x0d);
                      XuanZeDisplay(0xb0, 0xb1,0x17,0x00);
                      g_Key1StateCheck=0;
                      g_ChooseNumber1=1;
                                    }
                    if((g_Key1StateCheck==1)&&(g_ChooseNumber1==1)) {               
                      g_BackCounter1=0;
                                    ClearDisplay(0xb0, 0xb1,0x17,0x00);
                                XuanZeDisplay(0xb2, 0xb3,0x12,0x0d);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber1=2;
                                    }
                    if((g_Key1StateCheck==1)&&(g_ChooseNumber1==2)) {               
                      g_BackCounter1=0;
                                    ClearDisplay(0xb2,0xb3,0x12,0x0d);
                                XuanZeDisplay(0xb2, 0xb3,0x17,0x00);
                                    g_ChooseNumber1=3;
                                    g_Key1StateCheck=0;
                                    }	
                    if((g_Key1StateCheck==1)&&(g_ChooseNumber1==3)) {              
                      g_BackCounter1=0;
                                    ClearDisplay(0xb2, 0xb3,0x17,0x00);
                                XuanZeDisplay(0xb4, 0xb5,0x12,0x0d);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber1=4;
                                    }
                      if((g_Key1StateCheck==1)&&(g_ChooseNumber1==4)) {               
                        g_BackCounter1=0;
                                    ClearDisplay(0xb4, 0xb5,0x12,0x0d);
                                XuanZeDisplay(0xb4, 0xb5,0x17,0x00);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber1=5;
                                    }
                      if((g_Key1StateCheck==1)&&(g_ChooseNumber1==5)) {               
                        g_BackCounter1=0;
                                    ClearDisplay(0xb4, 0xb5,0x17,0x00);
                                XuanZeDisplay(0xb6, 0xb7,0x12,0x0d);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber1=6;
                                    }
                      if((g_Key1StateCheck==1)&&(g_ChooseNumber1==6)) {              
                        g_BackCounter1=0;
                                    ClearDisplay(0xb6, 0xb7,0x12,0x0d);
                                XuanZeDisplay(0xb6, 0xb7,0x17,0x00);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber1=7;
                                    }
                      if((g_Key1StateCheck==1)&&(g_ChooseNumber1==7)) {              
                        g_BackCounter1=0;
                                    ClearDisplay(0xb6, 0xb7,0x17,0x00);
                                XuanZeDisplay(0xb0, 0xb1,0x12,0x0d);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber1=0;
                                    }
                      if((g_Key2StateCheck==1)&&(g_ChooseNumber1==0)) {               
                        g_BackCounter1=0;
                      if(g_OverVoltage>260) {g_OverVoltage=260;}
                      if(g_OverVoltage<240) {g_OverVoltage=240;}
                      if(g_LowerVoltage>195) {g_LowerVoltage=195;}
                      if(g_LowerVoltage<175) {g_LowerVoltage=175;}
                      if(g_OverTemperature>70) {g_OverTemperature=70;}
                      if(g_OverTemperature<55) {g_OverTemperature=55;}
                      g_Key2StateCheck=0;
                      Display2_3Page();
                      LCD_Display(Bmp_XuanZe, 0xb0, 0x15,0x00, 16);
                      LCD_Display(Bmp_XuanZe+16, 0xb1, 0x15,0x00, 16);
                      while(1) { 	Delay(0x00006fff);     
                                       g_BackCounter1=g_BackCounter1+1;
                                       if(g_BackCounter1>100)
                                        {    g_BackCounter1=0;
                                              g_ChooseNumber2=0;
                                              break;
                                         }
                                      if((g_Key1StateCheck==1)&&(g_ChooseNumber2==0)) {              g_BackCounter1=0;
                                                    //  LCD_Display(Bmp_Clear16, 0xb0, 0x15,0x00, 16);
                                                   //   LCD_Display(Bmp_Clear16+16, 0xb1, 0x15,0x00, 16);
                                                   //   LCD_Display(Bmp_XuanZe, 0xb2, 0x15,0x00, 16);
                                                   //   LCD_Display(Bmp_XuanZe+16, 0xb3, 0x15,0x00, 16);
                                                                              ClearDisplay(0xb0, 0xb1,0x15,0x00);
                                                                   XuanZeDisplay(0xb2, 0xb3,0x15,0x00);
                                                      g_Key1StateCheck=0;
                                                      g_ChooseNumber2=1;
                                                      }
                                      if((g_Key1StateCheck==1)&&(g_ChooseNumber2==1)) {              g_BackCounter1=0;
                                                      //LCD_Display(Bmp_Clear16, 0xb2, 0x15,0x00, 16);
                                                     // LCD_Display(Bmp_Clear16+16, 0xb3, 0x15,0x00, 16);
                                                      //LCD_Display(Bmp_XuanZe, 0xb4, 0x15,0x00, 16);
                                                      //LCD_Display(Bmp_XuanZe+16, 0xb5, 0x15,0x00, 16);
                                                      ClearDisplay(0xb2, 0xb3,0x15,0x00);
                                               XuanZeDisplay(0xb4, 0xb5,0x15,0x00);
                                                      g_Key1StateCheck=0;
                                                      g_ChooseNumber2=2;
                                                      }
                              
                                      if((g_Key1StateCheck==1)&&(g_ChooseNumber2==2)) {              g_BackCounter1=0;
                                                      //LCD_Display(Bmp_Clear16, 0xb4, 0x15,0x00, 16);
                                                      //LCD_Display(Bmp_Clear16+16, 0xb5, 0x15,0x00, 16);
                                                      //LCD_Display(Bmp_XuanZe, 0xb6, 0x16,0x0e, 16);
                                                      //LCD_Display(Bmp_XuanZe+16, 0xb7, 0x16,0x0e, 16);
                                                      ClearDisplay(0xb4, 0xb5,0x15,0x00);
                                                                   XuanZeDisplay(0xb6, 0xb7,0x16,0x0e);
                                                      g_Key1StateCheck=0;
                                                      g_ChooseNumber2=3;
                                                      }
                                      if((g_Key1StateCheck==1)&&(g_ChooseNumber2==3)) {              g_BackCounter1=0;	
                                                      //LCD_Display(Bmp_Clear16, 0xb6, 0x16,0x0e, 16);
                                                      //LCD_Display(Bmp_Clear16+16, 0xb7, 0x16,0x0e, 16);
                                                      //LCD_Display(Bmp_XuanZe, 0xb0, 0x15,0x00, 16);
                                                      //LCD_Display(Bmp_XuanZe+16, 0xb1, 0x15,0x00, 16);
                                                      ClearDisplay(0xb6, 0xb7,0x16,0x0e);
                                                                   XuanZeDisplay(0xb0, 0xb1,0x15,0x00);
                                                      g_Key1StateCheck=0;
                                                      g_ChooseNumber2=0;
                                                      }
                                      if((g_Key2StateCheck==1)&&(g_ChooseNumber2==0)) {              g_BackCounter1=0;
                                                      g_Key2StateCheck=0;
                                                      g_OverVoltage=g_OverVoltage+1;
                                                      if(g_OverVoltage>260)  {g_OverVoltage=240;}
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverVoltage, 1), 0xb0, 0x13,0x00, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverVoltage, 1)+8, 0xb1, 0x13,0x00, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverVoltage, 2), 0xb0, 0x13,0x08, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverVoltage, 2)+8, 0xb1, 0x13,0x08, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverVoltage, 3), 0xb0, 0x14,0x00, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverVoltage, 3)+8, 0xb1, 0x14,0x00, 8);
                                                      }
                                      if((g_Key2StateCheck==1)&&(g_ChooseNumber2==1)) {              g_BackCounter1=0;
                                                      g_Key2StateCheck=0;
                                                      g_OverTemperature=g_OverTemperature+1;
                                                      if(g_OverTemperature>70) {g_OverTemperature=55;}
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverTemperature, 2), 0xb2, 0x13,0x04, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverTemperature, 2)+8, 0xb3, 0x13,0x04, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverTemperature, 3), 0xb2, 0x13,0x0c, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_OverTemperature, 3)+8, 0xb3, 0x13,0x0c, 8);
                                                      }
                                      if((g_Key2StateCheck==1)&&(g_ChooseNumber2==2)) {              g_BackCounter1=0;
                                                      g_Key2StateCheck=0;
                                                      g_LowerVoltage=g_LowerVoltage+1;
                                                      if(g_LowerVoltage>195) {g_LowerVoltage=175;}
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_LowerVoltage, 1), 0xb4, 0x13,0x00, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_LowerVoltage, 1)+8, 0xb5, 0x13,0x00, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_LowerVoltage, 2), 0xb4, 0x13,0x08, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_LowerVoltage, 2)+8, 0xb5, 0x13,0x08, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_LowerVoltage, 3), 0xb4, 0x14,0x00, 8);
                                                      LCD_Display(ChangeLCD_Data((uint16_t)g_LowerVoltage, 3)+8, 0xb5, 0x14,0x00, 8);
                                                      }
                                      if((g_Key2StateCheck==1)&&(g_ChooseNumber2==3)) {              g_BackCounter1=0;
                                                      g_Key2StateCheck=0;
                                                      g_ChooseNumber2=0;
                                                      break;
                                                      }
                                      Manu2AutoChange();
                                      if(g_AutoManual==0) {break;}
                                    }
                                    WriteEEPROM( OVER_VOLTAGE-2, g_OverVoltage);
                                    WriteEEPROM( LOWR_VOLTAGE-3, g_LowerVoltage);
                                    WriteEEPROM( OVER_TEMPERA-3, g_OverTemperature);
                                                    Display2_1Page();		
                                                    XuanZeDisplay(0xb0, 0xb1,0x12,0x0d);
                                    g_Key2StateCheck=0;
                                    nop();
                            }
                                if((g_Key2StateCheck==1)&&(g_ChooseNumber1==1)) 
                                  {	g_BackCounter1=0;
                                          g_Key2StateCheck=0;
                                          u8 g_M1=0;
                                          u8 g_M2=0;
                                          u8 g_M3=0;

                                          Display3_5Page(g_M1,g_M2,g_M3);
                                          //LCD_Display(Bmp_XuanZe, 0xb0, 0x10,0x08, 16);
                                          //LCD_Display(Bmp_XuanZe+16, 0xb1, 0x10,0x08, 16);
                                          XuanZeDisplay(0xb0, 0xb1,0x10,0x08);
                                          while(1) { 	  Delay(0x00006fff);     
                                          g_BackCounter1=g_BackCounter1+1;
                                 if(g_BackCounter1>100)
                                  {    g_BackCounter1=0;
                                        g_ChooseNumber3_4=0;
                                        break;
                                   }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_4==0)) {g_BackCounter1=0;
                                          ClearDisplay(0xb0, 0xb1,0x10,0x08);
                                          XuanZeDisplay(0xb0, 0xb1,0x12,0x08);
                                          g_Key1StateCheck=0;
                                          g_ChooseNumber3_4=1;
                                  }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_4==1)) {g_BackCounter1=0;
                                          ClearDisplay(0xb0, 0xb1,0x12,0x08);
                                          XuanZeDisplay(0xb0, 0xb1,0x14,0x08);
                                          g_Key1StateCheck=0;
                                          g_ChooseNumber3_4=2;
                                  }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_4==2)) {g_BackCounter1=0;
                                          ClearDisplay(0xb0, 0xb1,0x14,0x08);
                                          XuanZeDisplay(0xb2, 0xb3,0x14,0x00);
                                          g_Key1StateCheck=0;
                                          g_ChooseNumber3_4=3;
                                  }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_4==3)) {g_BackCounter1=0;
                                          ClearDisplay(0xb2, 0xb3,0x14,0x00);
                                          XuanZeDisplay(0xb6, 0xb7,0x14,0x00);
                                          g_Key1StateCheck=0;
                                          g_ChooseNumber3_4=4;
                                  }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_4==4)) {g_BackCounter1=0;
                                          ClearDisplay(0xb6, 0xb7,0x14,0x00);
                                          XuanZeDisplay(0xb0, 0xb1,0x10,0x08);
                                          g_Key1StateCheck=0;
                                          g_ChooseNumber3_4=0;
                                  }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_4==0)) {g_BackCounter1=0;
                                          g_Key2StateCheck=0;
                                          g_M1=g_M1+1;
                                          if(g_M1==10) {g_M1=0;}
                                          LCD_Display(ChangeLCD_Data((uint16_t)g_M1, 3), 0xb0, 0x10,0x00, 8);
                                          LCD_Display(ChangeLCD_Data((uint16_t)g_M1, 3)+8, 0xb1, 0x10,0x00, 8);
                                          
                                  }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_4==1)) {g_BackCounter1=0;
                                          g_Key2StateCheck=0;
                                          g_M2=g_M2+1;
                                          if(g_M2==10) {g_M2=0;}
                                          LCD_Display(ChangeLCD_Data((uint16_t)g_M2, 3), 0xb0, 0x12,0x00, 8);
                                          LCD_Display(ChangeLCD_Data((uint16_t)g_M2, 3)+8, 0xb1, 0x12,0x00, 8);
                                  }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_4==2)) {g_BackCounter1=0;
                                          g_Key2StateCheck=0;
                                          g_M3=g_M3+1;
                                          if(g_M3==10) {g_M3=0;}
                                          LCD_Display(ChangeLCD_Data((uint16_t)g_M3, 3), 0xb0, 0x14,0x00, 8);
                                          LCD_Display(ChangeLCD_Data((uint16_t)g_M3, 3)+8, 0xb1, 0x14,0x00, 8);
                                  }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_4==3)) {g_BackCounter1=0;
                                          g_Key2StateCheck=0;
                                          if((g_M1==1)&&(g_M2==1)&&(g_M3==1))
                                                  {	
                                                          LCD_Clear();
                                                          Display3_6Page();
                                                          //LCD_Display(Bmp_XuanZe, 0xb0, 0x14,0x00, 16);
                                                          //LCD_Display(Bmp_XuanZe+16, 0xb1, 0x14,0x00, 16);
                                                          
                                          XuanZeDisplay(0xb0, 0xb1,0x11,0x00);
                                                              
                            g_Key2StateCheck=0;
                            
                            if(g_CounterPointA1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointA1=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointA1<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointA1=(u16)((u32)SAMPLING_POINTS*40/100);}
                            if(g_CounterPointB1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointB1=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointB1<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointB1=(u16)((u32)SAMPLING_POINTS*40/100);}
                            if(g_CounterPointC1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointC1=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointC1<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointC1=(u16)((u32)SAMPLING_POINTS*40/100);}
                                              if(g_CounterPointA2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointA2=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointA2<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointA2=(u16)((u32)SAMPLING_POINTS*40/100);}
                            if(g_CounterPointB2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointB2=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointB2<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointB2=(u16)((u32)SAMPLING_POINTS*40/100);}
                            if(g_CounterPointC2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointC2=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointC2<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointC2=(u16)((u32)SAMPLING_POINTS*40/100);}
                                              if(g_CounterPointD1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointD1=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointD1<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointD1=(u16)((u32)SAMPLING_POINTS*40/100);}
                                              if(g_CounterPointD2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointD2=(u16)((u32)SAMPLING_POINTS*98/100);}
                            if(g_CounterPointD2<(u16)((u32)SAMPLING_POINTS*40/100)) {g_CounterPointD2=(u16)((u32)SAMPLING_POINTS*40/100);}
                            while(1)
                                    { 	  Delay(0x00006fff);     
                          g_BackCounter1=g_BackCounter1+1;
                          if(g_BackCounter1>100)
                                {    g_BackCounter1=0;
                                      g_ChooseNumber3=0;
                                      break;
                                 }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==0)) {g_BackCounter1=0;
                                       // LCD_Display(Bmp_Clear16, 0xb0, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb1, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb0, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb1, 0x16,0x00, 16);
                                                            ClearDisplay(0xb0, 0xb1,0x11,0x00);
                                                      XuanZeDisplay(0xb2, 0xb3,0x11,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=1;
                                                            }
                           if((g_Key1StateCheck==1)&&(g_ChooseNumber3==1)) {g_BackCounter1=0;
                                       //  LCD_Display(Bmp_Clear16, 0xb0, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb1, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x14,0x00, 16);
                                                            ClearDisplay(0xb2, 0xb3,0x11,0x00);
                                                      XuanZeDisplay(0xb0, 0xb1,0x13,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=2;
                                                            }
                           if((g_Key1StateCheck==1)&&(g_ChooseNumber3==2)) {g_BackCounter1=0;
                                      //  LCD_Display(Bmp_Clear16, 0xb2, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x16,0x00, 16);
                                                            ClearDisplay(0xb0, 0xb1,0x13,0x00);
                                                      XuanZeDisplay(0xb2, 0xb3,0x13,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=3;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==3)) {g_BackCounter1=0;
                                    //    LCD_Display(Bmp_Clear16, 0xb2, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb4, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb5, 0x14,0x00, 16);
                                                            ClearDisplay(0xb2, 0xb3,0x13,0x00);
                                                      XuanZeDisplay(0xb0, 0xb1,0x15,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=4;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==4)) {g_BackCounter1=0;
                                    //    LCD_Display(Bmp_Clear16, 0xb2, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb4, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb5, 0x14,0x00, 16);
                                                            ClearDisplay(0xb0, 0xb1,0x15,0x00);
                                                      XuanZeDisplay(0xb2, 0xb3,0x15,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=5;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==5)) {g_BackCounter1=0;
                                    //    LCD_Display(Bmp_Clear16, 0xb2, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb4, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb5, 0x14,0x00, 16);
                                                            ClearDisplay(0xb2, 0xb3,0x15,0x00);
                                                      XuanZeDisplay(0xb0, 0xb1,0x17,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=6;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==6)) {g_BackCounter1=0;
                                    //    LCD_Display(Bmp_Clear16, 0xb2, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x16,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb4, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb5, 0x14,0x00, 16);
                                                            ClearDisplay(0xb0, 0xb1,0x17,0x00);
                                                      XuanZeDisplay(0xb2, 0xb3,0x17,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=7;
                                                            }
                                    
                                            
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==7)) {g_BackCounter1=0;
                                    //    LCD_Display(Bmp_Clear16, 0xb4, 0x16,0x00, 16);
                                                    //	LCD_Display(Bmp_Clear16+16, 0xb5, 0x16,0x00, 16);
                                                    //	LCD_Display(Bmp_XuanZe, 0xb6, 0x16,0x0, 16);
                                                    //	LCD_Display(Bmp_XuanZe+16, 0xb7, 0x16,0x00, 16);
                                                            ClearDisplay(0xb2, 0xb3,0x17,0x00);
                                                      XuanZeDisplay(0xb6, 0xb7,0x15,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=8;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==8)) {g_BackCounter1=0;
                                     //   LCD_Display(Bmp_Clear16, 0xb6, 0x16,0x00, 16);
                                                    //	LCD_Display(Bmp_Clear16+16, 0xb7, 0x16,0x00, 16);
                                                    //	LCD_Display(Bmp_XuanZe, 0xb6, 0x12,0x00, 16);
                                                    //	LCD_Display(Bmp_XuanZe+16, 0xb7, 0x12,0x00, 16);
                                                            ClearDisplay(0xb6, 0xb7,0x15,0x00);
                                                      XuanZeDisplay(0xb6, 0xb7,0x16,0x08);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=9;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==9)) {g_BackCounter1=0;
                                     //   LCD_Display(Bmp_Clear16, 0xb6, 0x16,0x00, 16);
                                                    //	LCD_Display(Bmp_Clear16+16, 0xb7, 0x16,0x00, 16);
                                                    //	LCD_Display(Bmp_XuanZe, 0xb6, 0x12,0x00, 16);
                                                    //	LCD_Display(Bmp_XuanZe+16, 0xb7, 0x12,0x00, 16);
                                                            ClearDisplay(0xb6, 0xb7,0x16,0x08);
                                                      XuanZeDisplay(0xb6, 0xb7,0x12,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=10;
                                                            }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3==10)) {g_BackCounter1=0;
                                       // LCD_Display(Bmp_Clear16, 0xb6, 0x12,0x00, 16);
                                                            //LCD_Display(Bmp_Clear16+16, 0xb7, 0x12,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe, 0xb0, 0x14,0x00, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb1, 0x14,0x00, 16);
                                                            ClearDisplay(0xb6, 0xb7,0x12,0x00);
                                                      XuanZeDisplay(0xb0, 0xb1,0x11,0x00);
                                                            g_Key1StateCheck=0;
                                                            g_ChooseNumber3=0;
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==0)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointA1=g_CounterPointA1+1;
                                                            if(g_CounterPointA1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointA1=(u16)((u32)SAMPLING_POINTS*40/100);}
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA1, 1), 0xb0, 0x10,0x00, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA1, 1)+8, 0xb1, 0x10,0x00, 8);
                                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA1, 2), 0xb0, 0x10,0x00, 8);
                                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA1, 2)+8, 0xb1, 0x10,0x00, 8);
                                                                     LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA1, 3), 0xb0, 0x10,0x08, 8);
                                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA1, 3)+8, 0xb1, 0x10,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==1)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointA2=g_CounterPointA2+1;
                                                            if(g_CounterPointA2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointA2=(u16)((u32)SAMPLING_POINTS*40/100);}
                                //        LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA2, 1), 0xb0, 0x12,0x08, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA2, 1)+8, 0xb1, 0x12,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA2, 2), 0xb2, 0x10,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA2, 2)+8, 0xb3, 0x10,0x00, 8);
                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA2, 3), 0xb2, 0x10,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointA2, 3)+8, 0xb3, 0x10,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==2)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointB1=g_CounterPointB1+1;
                                                            if(g_CounterPointB1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointB1=(u16)((u32)SAMPLING_POINTS*40/100);}
                                  //       LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB1, 1), 0xb0, 0x15,0x00, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB1, 1)+8, 0xb1, 0x15,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB1, 2), 0xb0, 0x12,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB1, 2)+8, 0xb1, 0x12,0x00, 8);
                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB1, 3), 0xb0, 0x12,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB1, 3)+8, 0xb1, 0x12,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==3)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointB2=g_CounterPointB2+1;
                                                            if(g_CounterPointB2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointB2=(u16)((u32)SAMPLING_POINTS*40/100);}
                                  //      LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB2, 1), 0xb2, 0x10,0x00, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB2, 1)+8, 0xb3, 0x10,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB2, 2), 0xb2, 0x12,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB2, 2)+8, 0xb3, 0x12,0x00, 8);
                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB2, 3), 0xb2, 0x12,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointB2, 3)+8, 0xb3, 0x12,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==4)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointC1=g_CounterPointC1+1;
                                                            if(g_CounterPointC1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointC1=(u16)((u32)SAMPLING_POINTS*40/100);}
                                  //       LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC1, 1), 0xb2, 0x12,0x08, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC1, 1)+8, 0xb3, 0x12,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC1, 2), 0xb0, 0x14,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC1, 2)+8, 0xb1, 0x14,0x00, 8);
                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC1, 3), 0xb0, 0x14,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC1, 3)+8, 0xb1, 0x14,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==5)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointC2=g_CounterPointC2+1;
                                                            if(g_CounterPointC2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointC2=(u16)((u32)SAMPLING_POINTS*40/100);}
                               //         LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC2, 1), 0xb2, 0x15,0x00, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC2, 1)+8, 0xb3, 0x15,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC2, 2), 0xb2, 0x14,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC2, 2)+8, 0xb3, 0x14,0x00, 8);
                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC2, 3), 0xb2, 0x14,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointC2, 3)+8, 0xb3, 0x14,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==6)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointD1=g_CounterPointD1+1;
                                                            if(g_CounterPointD1>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointD1=(u16)((u32)SAMPLING_POINTS*40/100);}
                                //        LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD1, 1), 0xb4, 0x10,0x00, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD1, 1)+8, 0xb5, 0x10,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD1, 2), 0xb0, 0x16,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD1, 2)+8, 0xb1, 0x16,0x00, 8);
                                                    LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD1, 3), 0xb0, 0x16,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD1, 3)+8, 0xb1, 0x16,0x08, 8);
                                                            }
                                                    if((g_Key2StateCheck==1)&&(g_ChooseNumber3==7)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_CounterPointD2=g_CounterPointD2+1;
                                                            if(g_CounterPointD2>(u16)((u32)SAMPLING_POINTS*98/100)) {g_CounterPointD2=(u16)((u32)SAMPLING_POINTS*40/100);}
                                //         LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD2, 1), 0xb4, 0x12,0x08, 8);
                                                    //	LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD2, 1)+8, 0xb5, 0x12,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD2, 2), 0xb2, 0x16,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD2, 2)+8, 0xb3, 0x16,0x00, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD2, 3), 0xb2, 0x16,0x08, 8);
                                                            LCD_Display(ChangeLCD_Data((uint16_t)g_CounterPointD2, 3)+8, 0xb3, 0x16,0x08, 8);
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==9)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            
                                                            if((g_VoltageA>=210)&&(g_VoltageA<=250))
                                                            {	
                                                              if((g_VoltageA>=210)&&(g_VoltageA<=230))
                                                            {g_VoltageModifyA=g_VoltageModifyA+(230-g_VoltageA)*5/2;}		
                                                            if((g_VoltageA>230)&&(g_VoltageA<=250))			
                                                            {g_VoltageModifyA=g_VoltageModifyA-(g_VoltageA-230)*5/2;}	
                                                            WriteEEPROM( VA_OFFSET-2, g_VoltageModifyA);}  
                                                            
                                                            if((g_VoltageB>=210)&&(g_VoltageB<=250))	
                                                            {if((g_VoltageB>=210)&&(g_VoltageB<=230))	
                                                            {g_VoltageModifyB=g_VoltageModifyB+(230-g_VoltageB)*5/2;}	
                                                            if((g_VoltageB>230)&&(g_VoltageB<=250))	
                                                            {g_VoltageModifyB=g_VoltageModifyB-(g_VoltageB-230)*5/2;}	
                                                            WriteEEPROM( VB_OFFSET-2, g_VoltageModifyB);	
                                                            }
                                                            
                                                            if((g_VoltageC>=210)&&(g_VoltageC<=250))	
                                                            {if((g_VoltageC>=210)&&(g_VoltageC<=230))
                                                            {g_VoltageModifyC=g_VoltageModifyC+(230-g_VoltageC)*5/2;}	
                                                            if((g_VoltageC>230)&&(g_VoltageC<=250))	
                                                            {g_VoltageModifyC=g_VoltageModifyC-(g_VoltageC-230)*5/2;}		
                                                            WriteEEPROM( VC_OFFSET-2, g_VoltageModifyC);		
                                                            }
                                  
                                  if((g_VoltageA<210)||(g_VoltageA>250)||(g_VoltageB<210)||(g_VoltageB>250)||(g_VoltageC<210)||(g_VoltageC>250))
                                  {ClearDisplay(0xb6, 0xb7,0x16,0x00);									
                                  ClearDisplay(0xb6, 0xb7,0x17,0x00);									
                                  LCD_Display(Bmp_E, 0xb6, 0x16,0x00, 8);//U								
                                  LCD_Display(Bmp_E+8, 0xb7, 0x16,0x00, 8);								
                                  g_ChooseNumber3=10;											
                                 				                        
                                  XuanZeDisplay(0xb6, 0xb7,0x12,0x00);    }
                                  
                                  else
                                  {									
                                    ClearDisplay(0xb6, 0xb7,0x16,0x00);									
                                    ClearDisplay(0xb6, 0xb7,0x17,0x00);									
                                    LCD_Display(Bmp_OK, 0xb6, 0x16,0x00, 16);//U							
                                    LCD_Display(Bmp_OK+16, 0xb7, 0x16,0x00, 16);							
                                  
                                    g_ChooseNumber3=10;											
                                    //ClearDisplay(0xb6, 0xb7,0x16,0x08);
                                    
                                      XuanZeDisplay(0xb6, 0xb7,0x12,0x00);													
                                      }         
                                                                                                                     
                                     }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==10)) {g_BackCounter1=0;
                                                            g_Key2StateCheck=0;
                                                            g_ChooseNumber3=0;
                                                            g_M1=0;
                                                            g_M2=0;
                                                            g_M3=0;
                                                            break;
                                                            }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3==8)) 
                                            {
                                                g_BackCounter1=0;
                                                g_Key2StateCheck=0;
                                                DisableKey();
                                                                        
                                                
                                                if(g_SepCom==0)
                                                 {
                                                          g_Debug=1;
                                                         EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                          EnableCapA();
                                                          Delay(0x0002ffff);
                                                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                          g_Debug=1;
                                                          EnableCapB();																								
                                                          Delay(0x0002ffff);
                                                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                          g_Debug=1;
                                                          EnableCapC();
                                                          Delay(0x0002ffff);
                                                          g_Debug=0; 
                                                          DisableCap();
                                                  }
                                                 else
                                                 {
                                                    g_Debug=1;
                                                    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                    EnableCapC();
                                                 }
                                                          
                                                  Delay(0x0002ffff);	
                                                  g_M1=0;
                                                  g_M2=0;
                                                  g_M3=0;
                                                  LCD_Display(Bmp_OK, 0xb6, 0x16,0x00, 16);
                                                  LCD_Display(Bmp_OK+16, 0xb7, 0x16,0x00, 16);
                                                  EnableKey();
                                                  g_Debug=0; 
                                            }

                                            }
                                                            
                                          Display3_5Page(g_M1,g_M2,g_M3);
                                          //LCD_Display(Bmp_XuanZe, 0xb2, 0x14,0x0, 16);
                                          //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x14,0x00, 16);
                                          XuanZeDisplay(0xb2, 0xb3,0x14,0x00);
                                          }
                                          }
                                  
                                  

                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_4==4)) {g_BackCounter1=0;
                                    g_Key2StateCheck=0;
                                     g_ChooseNumber3_4=0;
                                    break;}
                            }
                    
                                            WriteEEPROM( A_REL_COUNTERH-2, g_CounterPointA1);
                                            WriteEEPROM( B_REL_COUNTERH-2, g_CounterPointB1);
                                            WriteEEPROM( C_REL_COUNTERH-2, g_CounterPointC1);
                                            WriteEEPROM( D_REL_COUNTERH-2, g_CounterPointD1);
                                            
                                            WriteEEPROM( A_REL_COUNTERL-2, g_CounterPointA2);
                                            WriteEEPROM( B_REL_COUNTERL-2, g_CounterPointB2);
                                            WriteEEPROM( C_REL_COUNTERL-2, g_CounterPointC2);
                                            WriteEEPROM( D_REL_COUNTERL-2, g_CounterPointD2);
                                            Display2_1Page();
                                         
                                            XuanZeDisplay(0xb0, 0xb1,0x17,0x00);
                                            g_Key2StateCheck=0;
                                            nop();
                                            }
                            
                            if((g_Key2StateCheck==1)&&(g_ChooseNumber1==2)) {               
                                            g_BackCounter1=0;
                                            LCD_Clear();
                                            LCD_Display(Bmp_DiZhi, 0xb2, 0x13,0x02, 32);
                                            LCD_Display(Bmp_DiZhi+32, 0xb3, 0x13,0x02, 32);
                                    
                                            IPAdressDisplay(g_IPAdress);
                                            LCD_Display(Bmp_TaiShu, 0xb4, 0x13,0x02, 32);
                                            LCD_Display(Bmp_TaiShu+32, 0xb5, 0x13,0x02, 32);
                                    
                                            EquipmentNumberDisplay( g_EquipmentNumber);
                                            LCD_Display(Bmp_FanHui, 0xb6, 0x13,0x02, 32);//返回
                                            LCD_Display(Bmp_FanHui+32, 0xb7, 0x13,0x02, 32);
                                            XuanZeDisplay(0xb2, 0xb3,0x16,0x08);
                                            g_Key2StateCheck=0;
                                            if(g_IPAdress>25) {g_IPAdress=1;}
                                            if(g_IPAdress==1) {g_IPAdress=1;}
                                            if(g_EquipmentNumber>25) {g_EquipmentNumber=1;}
                                            while(1) { 	 Delay(0x00006fff);     
                                                             g_BackCounter1=g_BackCounter1+1;
                                                             if(g_BackCounter1>100)
                                                              {    g_BackCounter1=0;
                                                                    g_ChooseNumber3_1=0;
                                                                    break;
                                                               }
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_1==0)) 
                                                            {g_BackCounter1=0;
                                                                            ClearDisplay(0xb2, 0xb3,0x16,0x08);
                                                                            XuanZeDisplay(0xb4, 0xb5,0x16,0x08);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_1=1;
                                                                            }
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_1==1)) 
                                                            {g_BackCounter1=0;
                                                                            ClearDisplay(0xb4, 0xb5,0x16,0x08);
                                                                            XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_1=2;
                                                                            }
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_1==2)) 
                                                            {g_BackCounter1=0;
                                                                            ClearDisplay(0xb6, 0xb7,0x15,0x03);
                                                                            XuanZeDisplay(0xb2, 0xb3,0x16,0x08);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_1=0;
                                                                            }
                                                    
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_1==0)) 
                                                            {g_BackCounter1=0;
                                                                            g_Key2StateCheck=0;
                                                                            g_IPAdress=g_IPAdress+1;
                                                                            if(g_IPAdress>25) {g_IPAdress=1;}
                                                                            IPAdressDisplay(g_IPAdress);
                                                                            }
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_1==1)) {g_BackCounter1=0;
                                                                            g_Key2StateCheck=0;
                                                                            g_EquipmentNumber=g_EquipmentNumber+1;
                                                                            if(g_EquipmentNumber>25) {g_EquipmentNumber=1;}
                                                                             EquipmentNumberDisplay( g_EquipmentNumber);
                                                                            }
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_1==2)) {g_BackCounter1=0;
                                                                            g_Key2StateCheck=0;
                                                                            g_ChooseNumber3_1=0;
                                                                            break;
                                                                            }
                                                          }	
                                                                    WriteEEPROM( ADDRESS-3, g_IPAdress);
                                                                    //手动设定地址后，清灯
                                                                    ERR_LED_OFF;
                                                                    Add_Alloc_Done=1;
                                                                    if((g_IPAdress>=1)&&(g_IPAdress<=14))   { UART1_SetAddress(g_IPAdress);}//IP地址写入ADD
                                                                    if((g_IPAdress>14)&&(g_IPAdress<=25))   { UART1_SetAddress(0x0f);}//IP地址写入ADD
                                                                    UART1->CR1 |= 0x08;                       //WAKE被地址标记唤醒
                                                                    UART1->CR2 |= (uint8_t)(UART1_CR2_RWU); //使能RWU
                                                                    WriteEEPROM( CAP_NUM-3, g_EquipmentNumber);
                                            Display2_1Page();
                                            XuanZeDisplay(0xb2, 0xb3,0x12,0x0d);
                                            g_Key2StateCheck=0;
                                            nop();
                                            }
                            if((g_Key2StateCheck==1)&&(g_ChooseNumber1==3)) 
                            {             
                                g_BackCounter1=0;
                                LCD_Clear();
                                if(g_MasterSlave>=2) 
                                  g_MasterSlave=0;
                                if(g_MasterSlave==1)
                                  {
                                      ZhuDisplay();
                                      MAS_LED_ON;
                                  }
                                else if(g_MasterSlave==0)
                                  {
                                      CongDisplay();
                                      MAS_LED_OFF;//GPIOC3 主机指示灯
                                  }
                                else
                                 {
                                   HouZhiDisplay();
                                 }
                             
                                if(g_SepCom==0)
                                   FenDisplay();
                                
                                if(g_SepCom==1)
                                  GongDisplay(); 
                             
                                LCD_Display(Bmp_FanHui, 0xb6, 0x13,0x02, 32);//返回
                                LCD_Display(Bmp_FanHui+32, 0xb7, 0x13,0x02, 32);
                                XuanZeDisplay(0xb2, 0xb3,0x15,0x05);
                                g_Key2StateCheck=0;
                                            
                                while(1) 
                                { 		
                                  Delay(0x00006fff);     
                                  g_BackCounter1=g_BackCounter1+1;
                                  if(g_BackCounter1>100)
                                    {    
                                      g_BackCounter1=0;
                                      g_ChooseNumber3_2=0;
                                      break;
                                     }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_2==0)) 
                                  {
                                    g_BackCounter1=0;
                                    ClearDisplay(0xb2, 0xb3,0x15,0x05);
                                    XuanZeDisplay(0xb4, 0xb5,0x15,0x05);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber3_2=1;
                                  }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_2==1)) 
                                  {
                                    g_BackCounter1=0;
                                    ClearDisplay(0xb4, 0xb5,0x15,0x05);
                                    XuanZeDisplay(0xb6, 0xb7,0x15,0x05);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber3_2=2;
                                  }
                                  if((g_Key1StateCheck==1)&&(g_ChooseNumber3_2==2)) 
                                  {
                                    g_BackCounter1=0;
                                    ClearDisplay(0xb6, 0xb7,0x15,0x05);
                                    XuanZeDisplay(0xb2, 0xb3,0x15,0x05);
                                    g_Key1StateCheck=0;
                                    g_ChooseNumber3_2=0;
                                   }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_2==0)) //主机从机侯址按键轮显
                                  {
                                    g_MasterSlave++;
                                    g_BackCounter1=0;
                                    if(g_MasterSlave==1)
                                      {
                                        ZhuDisplay();
                                        MAS_LED_ON;
                                      }
                                    else if(g_MasterSlave==0)
                                    {
                                      MAS_LED_OFF;
                                      CongDisplay();
                                    }
                                    else
                                    {
                                     HouZhiDisplay();
                                     MAS_LED_OFF;
                                      g_MasterSlave=-1;
                                    }
                                    g_Key2StateCheck=0;
                                  }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_2==1)) {g_BackCounter1=0;
                                                  if(g_SepCom==0)
                                                    {g_SepCom=1;
                                                
                                                    GongDisplay();
                                                                          
                                                    }
                                  else
                                                    {g_SepCom=0;
                                                                        
                                                    FenDisplay();
                                                    }
                                                  g_Key2StateCheck=0;
                                                  }
                                  if((g_Key2StateCheck==1)&&(g_ChooseNumber3_2==2)) {g_BackCounter1=0;
                                                  g_Key2StateCheck=0;
                                                  g_ChooseNumber3_2=0;
                                                  break;
                                                  }
                                              }	
                                          WriteEEPROM( MAS_SLA-3, g_MasterSlave);
                                          WriteEEPROM( COM_SEP-3, g_SepCom);
                                          Display2_1Page();
                                          XuanZeDisplay(0xb2, 0xb3,0x17,0x00);
                                          g_Key2StateCheck=0;
                            }
                            if((g_Key2StateCheck==1)&&(g_ChooseNumber1==4)) {               
                              g_BackCounter1=0;
                              LCD_Clear();
                               CTDisplay(g_CTPhaseA);
                                                           
                                            LCD_Display(Bmp_FanHui, 0xb6, 0x13,0x02, 32);//返回
                                            LCD_Display(Bmp_FanHui+32, 0xb7, 0x13,0x02, 32);
                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x12,0x08, 16);
                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x12,0x08, 16);
                                            XuanZeDisplay(0xb2, 0xb3,0x12,0x08);
                                            g_Key2StateCheck=0;
                                           
                                            while(1) { 		 Delay(0x00006fff);     
                                                             g_BackCounter1=g_BackCounter1+1;
                                                             if(g_BackCounter1>100)
                                                              {    g_BackCounter1=0;
                                                                    g_ChooseNumber3_3=0;
                                                                    break;
                                                               }
                                                             if(g_CTPhaseA>800) {g_CTPhaseA=10;}
                                    
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_3==0)) {g_BackCounter1=0;
                                                                         //   LCD_Display(Bmp_Clear16, 0xb2, 0x12,0x08, 16);
                                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x12,0x08, 16);
                                                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x14,0x08, 16);
                                                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x14,0x08, 16);
                                                                            ClearDisplay(0xb2, 0xb3,0x12,0x08);
                                                                                                XuanZeDisplay(0xb2, 0xb3,0x14,0x08);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_3=1;
                                                                            }
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_3==1)) {g_BackCounter1=0;
                                                                         //   LCD_Display(Bmp_Clear16, 0xb2, 0x14,0x08, 16);
                                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x14,0x08, 16);
                                                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x16,0x08, 16);
                                                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x16,0x08, 16);
                                                                            ClearDisplay(0xb2, 0xb3,0x14,0x08);
                                                                                                XuanZeDisplay(0xb2, 0xb3,0x16,0x08);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_3=2;
                                                                            }
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_3==2)) {g_BackCounter1=0;
                                                                        //    LCD_Display(Bmp_Clear16, 0xb2, 0x16,0x08, 16);
                                                                            //LCD_Display(Bmp_Clear16+16, 0xb3, 0x16,0x08, 16);
                                                                            //LCD_Display(Bmp_XuanZe, 0xb6, 0x15,0x03, 16);
                                                                            //LCD_Display(Bmp_XuanZe+16, 0xb7, 0x15,0x03, 16);
                                                                            ClearDisplay(0xb2, 0xb3,0x16,0x08);
                                                                                                XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_3=3;
                                                                            }
                                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_3==3)) {g_BackCounter1=0;
                                                                       //     LCD_Display(Bmp_Clear16, 0xb6, 0x15,0x03, 16);
                                                                            //LCD_Display(Bmp_Clear16+16, 0xb7, 0x15,0x03, 16);
                                                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x12,0x08, 16);
                                                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x12,0x08, 16);
                                                                            ClearDisplay(0xb6, 0xb7,0x15,0x03);
                                                                                                XuanZeDisplay(0xb2, 0xb3,0x12,0x08);
                                                                            g_Key1StateCheck=0;
                                                                            g_ChooseNumber3_3=0;
                                                                            }
                                                            
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_3==0)) {g_BackCounter1=0;
                                                                    //	g_Key2StateCheck=0;
                                                                            g_CTPhaseA=g_CTPhaseA+100;
                                                                            if(g_CTPhaseA>800) {g_CTPhaseA=10;}
                                                                            
                                                                                                                                                                                                                        CTDisplay(g_CTPhaseA);
                                                            g_Key2StateCheck=0;
                                                                            }
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_3==1)) {g_BackCounter1=0;
                                                                    //	g_Key2StateCheck=0;
                                                                            g_CTPhaseA=g_CTPhaseA+10;
                                                                            if(g_CTPhaseA>800) {g_CTPhaseA=10;}
                                                        CTDisplay(g_CTPhaseA);
                                                            g_Key2StateCheck=0;
                                                                            }
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_3==2)) {g_BackCounter1=0;
                                                                    //	g_Key2StateCheck=0;
                                                                            g_CTPhaseA=g_CTPhaseA+1;
                                                                            
                                                        CTDisplay(g_CTPhaseA);
                                                            g_Key2StateCheck=0;
                                                                            }
                                                    
                                    
                                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_3==3)) {g_BackCounter1=0;
                                                                            g_Key2StateCheck=0;
                                                                            g_ChooseNumber3_3=0;
                                                                            break;
                                                                            }
                                                          }	
                                                            WriteEEPROM( CT-2, g_CTPhaseA);
                                                    
                                         Display2_1Page();
                                       //  LCD_Display(Bmp_XuanZe, 0xb4, 0x12,0x0d, 16);
                                                            //LCD_Display(Bmp_XuanZe+16, 0xb5, 0x12,0x0d, 16);
                                                             XuanZeDisplay(0xb4, 0xb5,0x12,0x0d);
                                            g_Key2StateCheck=0;
                                            nop();
                                            }
                            if((g_Key2StateCheck==1)&&(g_ChooseNumber1==5)) {               g_BackCounter1=0;
                                            g_Key2StateCheck=0;
                                            LCD_Clear();
                                                    
                                            TotalCycleTimeDisplay(g_TotalCycleTimeNum);
                                            LCD_Display(Bmp_s, 0xb2, 0x14,0x04, 8);
                                            LCD_Display(Bmp_s+8, 0xb3, 0x14,0x04, 8);
                                            LCD_Display(Bmp_FanHui, 0xb6, 0x13,0x02, 32);//返回
                                            LCD_Display(Bmp_FanHui+32, 0xb7, 0x13,0x02, 32);
                                            //LCD_Display(Bmp_XuanZe, 0xb2, 0x16,0x04, 16);
                                            //LCD_Display(Bmp_XuanZe+16, 0xb3, 0x16,0x04, 16);
                                            XuanZeDisplay(0xb2, 0xb3,0x16,0x04);
                                            while(1) { 	 Delay(0x00006fff);     
                                                                     g_BackCounter1=g_BackCounter1+1;
                                                                     if(g_BackCounter1>100)
                                                                      {    g_BackCounter1=0;
                                                                            LCD_Clear();
                                                                            g_ChooseNumber3_5=0;
                                                                            break;
                                                                       }
                                                                    if((g_Key1StateCheck==1)&&(g_ChooseNumber3_5==0)) {	g_BackCounter1=0;
                                                                                                                           ClearDisplay(0xb2, 0xb3,0x16,0x04);
                                                                                                XuanZeDisplay(0xb6, 0xb7,0x16,0x04);
                                                                                                                            g_Key1StateCheck=0;
                                                                                                                            g_ChooseNumber3_5=1;
                                                                                                                            }
                                                                    if((g_Key1StateCheck==1)&&(g_ChooseNumber3_5==1)) {	g_BackCounter1=0;
                                                                                                                          ClearDisplay(0xb6, 0xb7,0x16,0x04);
                                                                                                XuanZeDisplay(0xb2, 0xb3,0x16,0x04);
                                                                                                                            g_Key1StateCheck=0;
                                                                                                                            g_ChooseNumber3_5=0;
                                                                                                                            }
                                                                    if((g_Key2StateCheck==1)&&(g_ChooseNumber3_5==0)) {g_BackCounter1=0;
                                                                                                    //	g_Key2StateCheck=0;
                                                                                                         if((g_TotalCycleTimeNum>=100)&&(g_TotalCycleTimeNum<=200))
                                                                                                            {g_TotalCycleTimeNum=g_TotalCycleTimeNum+50;} 
                                                                                                         if((g_TotalCycleTimeNum>=10)&&(g_TotalCycleTimeNum<100))
                                                                                                            {g_TotalCycleTimeNum=g_TotalCycleTimeNum+10;} 
                                                                                                          if((g_TotalCycleTimeNum>=4)&&(g_TotalCycleTimeNum<10))
                                                                                                            {g_TotalCycleTimeNum=g_TotalCycleTimeNum+1;}
                                                                                                             
                                                                                                            if(g_TotalCycleTimeNum>200) {g_TotalCycleTimeNum=4;}
                                                                                
                                                                               TotalCycleTimeDisplay(g_TotalCycleTimeNum);
                                                                                                            g_Key2StateCheck=0;
                                                                                                            }
                                                                    if((g_Key2StateCheck==1)&&(g_ChooseNumber3_5==1)) {g_BackCounter1=0;
                                                                                                            g_Key2StateCheck=0;
                                                                                     g_ChooseNumber3_5=0;
                                                                                                            break;
                                                                                                                    }
                                                                    
                                                    }
                                            WriteEEPROM( PERIOD-3, g_TotalCycleTimeNum);
                                            Display2_1Page();
                                            XuanZeDisplay(0xb4, 0xb5,0x17,0x00);
                                            nop();

                                            }
                            if((g_Key2StateCheck==1)&&(g_ChooseNumber1==6)) {               g_BackCounter1=0;
                                            g_Key2StateCheck=0;
                                            LCD_Clear();
                                            LCD_Display(Bmp_Tou, 0xb0, 0x12,0x00, 16);
                                            LCD_Display(Bmp_Tou+16, 0xb1, 0x12,0x00, 16);
                                            LCD_Display(Bmp_0, 0xb0, 0x13,0x00, 8);
                                            LCD_Display(Bmp_0+8, 0xb1, 0x13,0x00, 8);
                                            LCD_Display(Bmp_Dot, 0xb0, 0x13,0x08, 4);
                                            LCD_Display(Bmp_Dot+4, 0xb1, 0x13,0x08, 4);
                                           
                             PFTouDisplay(g_PFTou);	
                                            LCD_Display(Bmp_Qie, 0xb2, 0x12,0x00, 16);
                                            LCD_Display(Bmp_Qie+16, 0xb3, 0x12,0x00, 16);
                                            LCD_Display(Bmp_0, 0xb2, 0x13,0x00, 8);
                                            LCD_Display(Bmp_0+8, 0xb3, 0x13,0x00, 8);
                                            LCD_Display(Bmp_Dot, 0xb2, 0x13,0x08, 4);
                                            LCD_Display(Bmp_Dot+4, 0xb3, 0x13,0x08, 4);
                                          
                                            PFQieDisplay(g_PFQie);
                                            LCD_Display(Bmp_ReactivePower, 0xb4, 0x12,0x00, 8);
                                            LCD_Display(Bmp_ReactivePower+8, 0xb5, 0x12,0x00, 8);
                                           
                                            ReactiveQieDisplay(g_ReactiveQie);	
                                            LCD_Display(Bmp_kVar, 0xb4, 0x14,0x08, 32);
                                            LCD_Display(Bmp_kVar+32, 0xb5, 0x14,0x08, 32);
                                      
                            LCD_Display(Bmp_FanHui, 0xb6, 0x13,0x02, 32);//返回
                                            LCD_Display(Bmp_FanHui+32, 0xb7, 0x13,0x02, 32);
                                            //LCD_Display(Bmp_XuanZe, 0xb0, 0x16,0x08, 16);
                                            //LCD_Display(Bmp_XuanZe+16, 0xb1, 0x16,0x08, 16);
                                            XuanZeDisplay(0xb0, 0xb1,0x16,0x08);
                                            while(1) { 	
                                              Delay(0x00006fff);     
                                             g_BackCounter1=g_BackCounter1+1;
                                             if(g_BackCounter1>100)
                                              {    g_BackCounter1=0;
                                                    LCD_Clear();
                                                    g_ChooseNumber3_6=0;
                                                    break;
                                               }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_6==0)) {	g_BackCounter1=0;
                                                                                
                                                                                                    ClearDisplay(0xb0, 0xb1,0x16,0x08);
                                                                        XuanZeDisplay(0xb2, 0xb3,0x16,0x08);
                                                                                                    g_Key1StateCheck=0;
                                                                                                    g_ChooseNumber3_6=1;
                                                                                                    }
                                            
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_6==1)) {	g_BackCounter1=0;
                                                                                 
                                                                                                    ClearDisplay(0xb2, 0xb3,0x16,0x08);
                                                                        XuanZeDisplay(0xb4, 0xb5,0x16,0x08);
                                                                                                    g_Key1StateCheck=0;
                                                                                                    g_ChooseNumber3_6=2;
                                                                                                    }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_6==2)) {	g_BackCounter1=0;
                                                                            
                                                                                                    ClearDisplay(0xb4, 0xb5,0x16,0x08);
                                                                        XuanZeDisplay(0xb6, 0xb7,0x16,0x08);
                                                                                                    g_Key1StateCheck=0;
                                                                                                    g_ChooseNumber3_6=3;
                                                                                                    }
                                            if((g_Key1StateCheck==1)&&(g_ChooseNumber3_6==3)) {	g_BackCounter1=0;
                                                                                
                                                                                                    ClearDisplay(0xb6, 0xb7,0x16,0x08);
                                                                        XuanZeDisplay(0xb0, 0xb1,0x16,0x08);
                                                                                                    g_Key1StateCheck=0;
                                                                                                    g_ChooseNumber3_6=0;
                                                                                                    }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_6==0)) {g_BackCounter1=0;
                                                                                    g_PFTou=g_PFTou+10;
                                                                                    if(g_PFTou>960) {g_PFTou=920;}
                                                                                    
                                                                                     PFTouDisplay(g_PFTou);	
                                                                                    g_Key2StateCheck=0;
                                                                                    }
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_6==1)) {g_BackCounter1=0;
                                                                                    g_PFQie=g_PFQie+10;
                                                                                    if(g_PFQie>990) {g_PFQie=970;}
                                                                                    
                                                                                    PFQieDisplay(g_PFQie);
                                                                                    g_Key2StateCheck=0;
                                                                                    }
                            
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_6==2)) {g_BackCounter1=0;
                                                                                    g_ReactiveQie=g_ReactiveQie+5;
                                                                                    if(g_ReactiveQie>50) {g_ReactiveQie=0;}
                                                                                    
                                                         ReactiveQieDisplay(g_ReactiveQie);
                                                                                    g_Key2StateCheck=0;
                                                                                    }
                                            
                                            if((g_Key2StateCheck==1)&&(g_ChooseNumber3_6==3)) {g_BackCounter1=0;
                                                                                    g_Key2StateCheck=0;
                                                             g_ChooseNumber3_6=0;
                                                                                    break;
                                                                                            }
                                            
                            }
                                                                                WriteEEPROM( PF_LLIM-2, g_PFTou);
                                                                                WriteEEPROM( PF_HLIM-2, g_PFQie);
                                                                                WriteEEPROM( REACT_L-3, g_ReactiveQie);
                                                                                Display2_1Page();
                                                                                XuanZeDisplay(0xb6, 0xb7,0x12,0x0d);
                                                                                }
                                                                if((g_Key2StateCheck==1)&&(g_ChooseNumber1==7)) {               g_BackCounter1=0;
                                                                                g_Key2StateCheck=0;
                                                                                g_ChooseNumber1=0;
                                                                                break; }
                                                                        Manu2AutoChange();
                                                                        if(g_AutoManual==0) {break;}
                                                  }   
                                               Display1Page();
                                              //  LCD_Display(Bmp_XuanZe, 0xb4, 0x13,0x00, 16);
                                             //   LCD_Display(Bmp_XuanZe+16, 0xb5, 0x13,0x00, 16);
                                                                XuanZeDisplay(0xb4, 0xb5,0x13,0x00);
                                                g_Key2StateCheck=0;
                                                                     
                        }
                if((g_Key2StateCheck==1)&&(g_DisplayNumber==1)) {       g_BackCounter1=0;
                                Display2_2Page();	
                                
                                XuanZeDisplay(0xb0, 0xb1,0x16,0x0a);
                                
                                g_Key2StateCheck=0;
                                while(1) { 	 Delay(0x00001fff);     
                                                 g_BackCounter1=g_BackCounter1+1;
                                                 if(g_BackCounter1>500)
                                                  {    g_BackCounter1=0;
                                                        LCD_Clear();
                                                        g_ChooseNumber4=0;
                                                        break;
                                                   }
                                                if((g_Key1StateCheck==1)&&(g_ChooseNumber4==0)) {	g_BackCounter1=0;
                                                                                                        ClearDisplay(0xb0, 0xb1,0x16,0x0a);
                                                                                                        XuanZeDisplay(0xb2, 0xb3,0x16,0x0a);
                                                                                                        g_Key1StateCheck=0;
                                                                                                        g_ChooseNumber4=1;
                                                                                                        }
                                                if((g_Key1StateCheck==1)&&(g_ChooseNumber4==1)) {	g_BackCounter1=0;
                                                                                                        ClearDisplay(0xb2, 0xb3,0x16,0x0a);
                                                                                                        XuanZeDisplay(0xb4, 0xb5,0x16,0x0a);
                                                                                                        g_Key1StateCheck=0;
                                                                                                        g_ChooseNumber4=2;
                                                                                                        }
                                                if((g_Key1StateCheck==1)&&(g_ChooseNumber4==2)) {	g_BackCounter1=0;
                                                                                                        ClearDisplay(0xb4, 0xb5,0x16,0x0a);
                                                                                                        XuanZeDisplay(0xb6, 0xb7,0x12,0x02);
                                                                                                        g_Key1StateCheck=0;
                                                                                                        if((g_MasterSlave==1)&&(g_IPAdress==1))
                                                                                                        {g_ChooseNumber4=3;}
                                                                                                        else 
                                                                                                        {g_ChooseNumber4=5;}
                                                                                                        }
                                                if((g_Key1StateCheck==1)&&(g_ChooseNumber4==3)) {	g_BackCounter1=0;
                                                                                                        ClearDisplay(0xb6, 0xb7,0x12,0x02);
                                                                                                        XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                                                                        g_Key1StateCheck=0;
                                                                                                        g_ChooseNumber4=4;
                                                                                                        }
                                                if((g_Key1StateCheck==1)&&(g_ChooseNumber4==4)) {	g_BackCounter1=0;
                                                                                                        ClearDisplay(0xb6, 0xb7,0x15,0x03);
                                                                                                        XuanZeDisplay(0xb0, 0xb1,0x16,0x0a);
                                                                                                        g_Key1StateCheck=0;
                                                                                                        g_ChooseNumber4=0;
                                                                                                        }
                        
                                                if((g_Key1StateCheck==1)&&(g_ChooseNumber4==5)) {	g_BackCounter1=0;
                                                                                                        ClearDisplay(0xb6, 0xb7,0x12,0x02);
                                                                                                        XuanZeDisplay(0xb0, 0xb1,0x16,0x0a);
                                                                                                        g_Key1StateCheck=0;
                                                                                                        g_ChooseNumber4=0;
                                                                                                        }
                                                if((g_Key2StateCheck==1)&&(g_ChooseNumber4==0) ){g_BackCounter1=0;
                                                          if(g_PhaseATurn==1) {//a相G5，磁保持通GPIOG5为0，断为1
                                                                          g_Key2StateCheck=0;
                                                                          DisableKey();
                                                                          
                                                                          if(g_SepCom==0)
                                                                          { 
                                                                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                            g_PhaseATurn=0;  //A切除 
                                                                            EnableCapA();
                                                                            Delay(0x0005fff);
                                                                            LCD_Display(Bmp_Qie, 0xb0, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Qie+16, 0xb1, 0x14,0x0f, 16);
                                                                            DisableCap();
                                                                          }
                                                                          else if(g_SepCom==1)
                                                                          {
                                                                          g_PhaseATurn=0;  //四磁保持一起切除
                                                                          g_PhaseBTurn=0;
                                                                          g_PhaseCTurn=0;
                                                                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                          EnableCapC();//实际为AB电压中断
                                                                          Delay(0x002ffff);
                                                                         // DisableCap();
                                                                          LCD_Display(Bmp_Qie, 0xb0, 0x14,0x0f, 16);
                                                                          LCD_Display(Bmp_Qie+16, 0xb1, 0x14,0x0f, 16);
                                                                          DisableCap();
                                                                          }
                                                                          EnableKey();
                                                                          }
                                                          else if(g_PhaseATurn==0)
                                                                    {     g_Key2StateCheck=0;
                                                                          DisableKey();
                                                                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                          if(g_SepCom==0)
                                                                          { 
                                                                            g_PhaseATurn=1;   //A投入
                                                                            EnableCapAS();
                                                                            Delay(0x0005fff);
                                                                            LCD_Display(Bmp_Tou, 0xb0, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Tou+16, 0xb1, 0x14,0x0f, 16);
                                                                            DisableCap();}
                                                                          else if(g_SepCom==1)
                                                                          {
                                                                            g_PhaseATurn=1;   //A1投入
                                                                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                            EnableCapAS();//实际为A磁保持中断
                                                                            Delay(0x001ffff);
                                                                            DisableCap();
                                                                            Delay(0x0004fff);
                                                                            LCD_Display(Bmp_Tou, 0xb0, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Tou+16, 0xb1, 0x14,0x0f, 16);
                                                                            DisableCap();                                                                                                                                                                                                                                                                                                  
                                                                          }
                                                                          EnableKey(); 
                                                                          }
                                                          }
        if((g_Key2StateCheck==1)&&(g_ChooseNumber4==1) ){g_BackCounter1=0;
                                                          if(g_PhaseBTurn==1||g_PhaseCTurn==1) {
                                                                          g_Key2StateCheck=0;
                                                                          DisableKey();	
                                                                          if(g_SepCom==0)
                                                                            {
                                                                              EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效	
                                                                              g_PhaseBTurn=0;   //B切除
                                                                              EnableCapB(); 
                                                                              Delay(0x0007fff);
                                                                              LCD_Display(Bmp_Qie, 0xb2, 0x14,0x0f, 16);
                                                                              LCD_Display(Bmp_Qie+16, 0xb3, 0x14,0x0f, 16);
                                                                              DisableCap();                                                                                                                                                                                                                                                                                              
                                                                            }
                                                                          else if(g_SepCom==2) //手动投切 界面C2功能不打开
                                                                          {
                                                                            g_PhaseATurn=0;  
                                                                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                            EnableCapA();
                                                                            Delay(0x000ffff);
                                                                            DisableCap();
                                                                          
                                                                            Delay(0x0004fff);
                                                                            LCD_Display(Bmp_Qie, 0xb2, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Qie+16, 0xb3, 0x14,0x0f, 16);
                                                                            DisableCap();
                                                                          }
                                                                          EnableKey();
                                                                          }
                                                          else {	        g_Key2StateCheck=0;
                                                                          DisableKey();
                                                                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                          
                                                                          if(g_SepCom==0)
                                                                          {
                                                                            g_PhaseBTurn=1; //B2投入                                                                                                                                                  
                                                                            EnableCapBS();
                                                                            Delay(0x0007fff);
                                                                            LCD_Display(Bmp_Tou, 0xb2, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Tou+16, 0xb3, 0x14,0x0f, 16);
                                                                            DisableCap();
                                                                          }
                                                                          else if(g_SepCom==2)
                                                                          {
                                                                              g_PhaseCTurn=1;   //A2投入
                                                                             
                                                                              EnableCapCS();
                                                                              Delay(0x0ffff);
                                                                              DisableCap();
                                                                              Delay(0x0004fff);
                                                                              LCD_Display(Bmp_Tou, 0xb2, 0x14,0x0f, 16);
                                                                              LCD_Display(Bmp_Tou+16, 0xb3, 0x14,0x0f, 16);
                                                                              DisableCap();  
                                                                          }
                                                                            EnableKey();
                                                                          }
                                                          }	
        if((g_Key2StateCheck==1)&&(g_ChooseNumber4==2) ){g_BackCounter1=0;
                                                          if(g_PhaseCTurn==1) {
                                                                          g_Key2StateCheck=0;
                                                                          DisableKey();
                                                                          
                                                                          if(g_SepCom==0)
                                                                          {
                                                                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                            g_PhaseCTurn=0;  //C切除 
                                                                            EnableCapC();
                                                                            Delay(0x0005fff);
                                                                            LCD_Display(Bmp_Qie, 0xb4, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Qie+16, 0xb5, 0x14,0x0f, 16);
                                                                            DisableCap();
                                                                          }
                                                                           else
                                                                         {
                                                                            g_PhaseATurn=0;
                                                                           g_PhaseBTurn=0;
                                                                           g_PhaseCTurn=0;
                                                                           g_PhaseDTurn=0;
                                                                           ProtectAction();//全切除
                                                                          
                                                                         }
                                                                          EnableKey();
                                                                          }
                                                          else if(g_PhaseCTurn==0){	       
                                                                          g_Key2StateCheck=0;
                                                                          DisableKey();
                                                                         if(g_SepCom==0)
                                                                          {
                                                                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);//GPIOB中断上升沿有效
                                                                            g_PhaseCTurn=1;   //C投入
                                                                            EnableCapCS();
                                                                            Delay(0x0005fff);
                                                                            LCD_Display(Bmp_Tou, 0xb4, 0x14,0x0f, 16);
                                                                            LCD_Display(Bmp_Tou+16, 0xb5, 0x14,0x0f, 16);
                                                                            DisableCap();
                                                                          }
                                                                         else
                                                                         {
                                                                            g_PhaseATurn=0;
                                                                           g_PhaseBTurn=0;
                                                                           g_PhaseCTurn=0;
                                                                           g_PhaseDTurn=0;
                                                                           ProtectAction();// 全切除
                                                                          
                                                                         }
                                                                          
                                                                          EnableKey();
                                                                  }
                                                          }
          
                                                        if((g_Key2StateCheck==1)&&(g_ChooseNumber4==3)) 
                                                        {
                                                          g_BackCounter1=0;
                                                          g_Key2StateCheck=0;
                                                          g_ChooseNumber4=0;
                                                          break;	}

                                                        if((g_Key2StateCheck==1)&&(g_ChooseNumber4==4)&&(g_MasterSlave==1)&&(g_IPAdress==1)) 
                                                        {
                                                          g_BackCounter1=0;
                                                          //SlaveTestNum1=0;
                                                          g_Key2StateCheck=0;	
                                                          Display3_CongJiPage();
                                                          if(g_EquipmentNumber!=1)
                                                          { LCD_Display(Bmp_sXuanze, 0xb0, 0x11,0x00, 8);}
                                                          while(1) { 	 Delay(0x00006fff);     
                                                                           g_BackCounter1=g_BackCounter1+1;
                                                                           if(g_BackCounter1>100)
                                                                            {    g_BackCounter1=0;
                                                                                  LCD_Clear();
                                                                                  g_ChooseNumber5=2;
                                                                                  break;
                                                                             }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==2)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb0, 0x11,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb0, 0x12,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb0, 0xb0, 0x11, 0x00,0x12, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=3;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==3)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb0, 0x12,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb0, 0x14,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb0, 0xb0, 0x12, 0x08,0x14, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=4;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==4)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                             //     LCD_Display(Bmp_Clear8, 0xb0, 0x14,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb0, 0x15,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb0, 0xb0, 0x14, 0x00,0x15, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=5;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==5)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                               //   LCD_Display(Bmp_Clear8, 0xb0, 0x15,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb0, 0x17,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb0, 0xb0, 0x15, 0x08,0x17, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=6;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==6)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb0, 0x17,0x00, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb1, 0x11,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb0, 0xb1, 0x17, 0x00,0x11, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=7;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==7)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb1, 0x11,0x00, 8);
                                                                                                                          //LCD_Display(Bmp_sXuanze, 0xb1, 0x12,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb1, 0xb1, 0x11, 0x00,0x12, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=8;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==8)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb1, 0x12,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb1, 0x14,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb1, 0xb1, 0x12, 0x08,0x14, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=9;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==9)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb1, 0x14,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb1, 0x15,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb1, 0xb1, 0x14, 0x00,0x15, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=10;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==10)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb1, 0x15,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb1, 0x17,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb1, 0xb1, 0x15, 0x08,0x17, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=11;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==11)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                           //       LCD_Display(Bmp_Clear8, 0xb1, 0x17,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb2, 0x11,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb1, 0xb2, 0x17, 0x00,0x11, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=12;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==12)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb2, 0x11,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb2, 0x12,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb2, 0xb2, 0x11, 0x00,0x12, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=13;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==13)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb2, 0x12,0x08, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb2, 0x14,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb2, 0xb2, 0x12, 0x08,0x14, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=14;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==14)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb2, 0x14,0x00, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb2, 0x15,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb2, 0xb2, 0x14, 0x00,0x15, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=15;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==15)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                               //   LCD_Display(Bmp_Clear8, 0xb2, 0x15,0x08, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb2, 0x17,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb2, 0xb2, 0x15, 0x08,0x17, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=16;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==16)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                            //      LCD_Display(Bmp_Clear8, 0xb2, 0x17,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb3, 0x11,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb2, 0xb3, 0x17, 0x00,0x11, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=17;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==17)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                          //        LCD_Display(Bmp_Clear8, 0xb3, 0x11,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb3, 0x12,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb3, 0xb3, 0x11, 0x00,0x12, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=18;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==18)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb3, 0x12,0x08, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb3, 0x14,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb3, 0xb3, 0x12, 0x08,0x14, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=19;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==19)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb3, 0x14,0x00, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb3, 0x15,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb3, 0xb3, 0x14, 0x00,0x15, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=20;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==20)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb3, 0x15,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb3, 0x17,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb3, 0xb3, 0x15, 0x08,0x17, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=21;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==21)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb3, 0x17,0x00, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb4, 0x11,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb3, 0xb4, 0x17, 0x00,0x11, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=22;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==22)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb4, 0x11,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb4, 0x12,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb4, 0xb4, 0x11, 0x00,0x12, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=23;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==23)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb4, 0x12,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb4, 0x14,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb4, 0xb4, 0x12, 0x08,0x14, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=24;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==24)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb4, 0x14,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb4, 0x15,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb4, 0xb4, 0x14, 0x00,0x15, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=25;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==25)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear8, 0xb4, 0x15,0x08, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb4, 0x17,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb4, 0xb4, 0x15, 0x08,0x17, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=26;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==26)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb4, 0x17,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb5, 0x11,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb4, 0xb5, 0x17, 0x00,0x11, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=27;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==27)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb5, 0x11,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb5, 0x12,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb5, 0xb5, 0x11, 0x00,0x12, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=28;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==28)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                            //      LCD_Display(Bmp_Clear8, 0xb5, 0x12,0x08, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb5, 0x14,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb5, 0xb5, 0x12, 0x08,0x14, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=29;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==29)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                               //   LCD_Display(Bmp_Clear8, 0xb5, 0x14,0x00, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb5, 0x15,0x08, 8);
                                                                                                                                  ClearXuanZe8Display(0xb5, 0xb5, 0x14, 0x00,0x15, 0x08) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=30;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==30)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                              //    LCD_Display(Bmp_Clear8, 0xb5, 0x15,0x08, 8);
                                                                                                                          //	LCD_Display(Bmp_sXuanze, 0xb5, 0x17,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb5, 0xb5, 0x15, 0x08,0x17, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=31;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==31)&&(g_ChooseNumber5<g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                               //   LCD_Display(Bmp_Clear8, 0xb5, 0x17,0x00, 8);
                                                                                                                                  //LCD_Display(Bmp_sXuanze, 0xb6, 0x11,0x00, 8);
                                                                                                                                  ClearXuanZe8Display(0xb5, 0xb6, 0x17, 0x00,0x11, 0x00) ;
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=32;
                                                                                                                                  }
                                                                          
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==g_EquipmentNumber)) {	g_BackCounter1=0;
                                                                                                                                  if(g_ChooseNumber5==2) {LCD_Display(Bmp_Clear8, 0xb0, 0x11,0x00, 8);}
                                                                                                                  if(g_ChooseNumber5==3) {LCD_Display(Bmp_Clear8, 0xb0, 0x12,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==4) {LCD_Display(Bmp_Clear8, 0xb0, 0x14,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==5) {LCD_Display(Bmp_Clear8, 0xb0, 0x15,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==6) {LCD_Display(Bmp_Clear8, 0xb0, 0x17,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==7) {LCD_Display(Bmp_Clear8, 0xb1, 0x11,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==8) {LCD_Display(Bmp_Clear8, 0xb1, 0x12,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==9) {LCD_Display(Bmp_Clear8, 0xb1, 0x14,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==10) {LCD_Display(Bmp_Clear8, 0xb1, 0x15,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==11) {LCD_Display(Bmp_Clear8, 0xb1, 0x17,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==12) {LCD_Display(Bmp_Clear8, 0xb2, 0x11,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==13) {LCD_Display(Bmp_Clear8, 0xb2, 0x12,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==14) {LCD_Display(Bmp_Clear8, 0xb2, 0x14,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==15) {LCD_Display(Bmp_Clear8, 0xb2, 0x15,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==16) {LCD_Display(Bmp_Clear8, 0xb2, 0x17,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==17) {LCD_Display(Bmp_Clear8, 0xb3, 0x11,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==18) {LCD_Display(Bmp_Clear8, 0xb3, 0x12,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==19) {LCD_Display(Bmp_Clear8, 0xb3, 0x14,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==20) {LCD_Display(Bmp_Clear8, 0xb3, 0x15,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==21) {LCD_Display(Bmp_Clear8, 0xb3, 0x17,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==22) {LCD_Display(Bmp_Clear8, 0xb4, 0x11,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==23) {LCD_Display(Bmp_Clear8, 0xb4, 0x12,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==24) {LCD_Display(Bmp_Clear8, 0xb4, 0x14,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==25) {LCD_Display(Bmp_Clear8, 0xb4, 0x15,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==26) {LCD_Display(Bmp_Clear8, 0xb4, 0x17,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==27) {LCD_Display(Bmp_Clear8, 0xb5, 0x11,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==28) {LCD_Display(Bmp_Clear8, 0xb5, 0x12,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==29) {LCD_Display(Bmp_Clear8, 0xb5, 0x14,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==30) {LCD_Display(Bmp_Clear8, 0xb5, 0x15,0x08, 8);}
                                                                                                                                  if(g_ChooseNumber5==31) {LCD_Display(Bmp_Clear8, 0xb5, 0x17,0x00, 8);}
                                                                                                                                  if(g_ChooseNumber5==32) {LCD_Display(Bmp_Clear8, 0xb6, 0x11,0x00, 8);}
                                                                                                                                  //LCD_Display(Bmp_XuanZe, 0xb6, 0x15,0x03, 16);
                                                                                                                                  //LCD_Display(Bmp_XuanZe+16, 0xb7, 0x15,0x03, 16);
                                                                                                                                  XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=33;
                                                                                                                                  }
                                                                          if(g_EquipmentNumber==1)  {g_BackCounter1=0;
                                                                                                                                  //LCD_Display(Bmp_XuanZe, 0xb6, 0x15,0x03, 16);
                                                                                                                                  //LCD_Display(Bmp_XuanZe+16, 0xb7, 0x15,0x03, 16);
                                                                                                                                  XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=33;
                                                                                                                                  }
                                                                          if((g_Key1StateCheck==1)&&(g_ChooseNumber5==33)) {	g_BackCounter1=0;
                                                                                                                //  LCD_Display(Bmp_Clear16, 0xb6, 0x15,0x03, 16);
                                                                                                                                  //LCD_Display(Bmp_Clear16+16, 0xb7, 0x15,0x03, 16);
                                                                                                                                  XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb0, 0x11,0x00, 8);
                                                                                                                                  g_Key1StateCheck=0;
                                                                                                                                  g_ChooseNumber5=2;
                                                                                                                                  }
                                          
                                                                  
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==2) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(2, 0x82,0x82, 0x56, 0x58,0x5a, 0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb0, 0x11,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==3) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(3, 0x83,0x83, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb0, 0x12,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==4) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(4, 0x84, 0x84, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb0, 0x14,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==5) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(5, 0x85, 0x85, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb0, 0x15,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==6) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(6, 0x86, 0x86, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb0, 0x17,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==7) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(7, 0x87, 0x87, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb1, 0x11,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==8) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(8, 0x88,0x88,  0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb1, 0x12,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==9) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(9, 0x89, 0x89,0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb1, 0x14,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==10) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(10, 0x8a,0x8a, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb1, 0x15,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==11) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(11, 0x8b, 0x8b,0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb1, 0x17,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==12) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(12, 0x8c, 0x8c,0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb2, 0x11,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==13) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(13, 0x8d, 0x8d, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb2, 0x12,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==14) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(14, 0x8e,0x8e, 0x56, 0x58,0x5a,  0x61,0x62, 0x63,0x64, 0x65,0x66);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb2, 0x14,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==15) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(15, 0x8f,0x8f, 0x56, 0x58,0x5a, 0x71,0x31, 0x21,0x41, 0x11,0x01);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb2, 0x15,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==16) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(16, 0x9f, 0x9f, 0x56, 0x58,0x5a, 0x72,0x32, 0x22,0x42, 0x12,0x02);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb2, 0x17,0x00, 8);			
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==17) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(17, 0xaf, 0xaf, 0x56, 0x58,0x5a, 0x73,0x33, 0x23,0x43, 0x13,0x03);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb3, 0x11,0x00, 8);	
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==18) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(18, 0xbf, 0xbf, 0x56, 0x58,0x5a, 0x74,0x34, 0x24,0x44, 0x14,0x04);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb3, 0x12,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==19) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(19, 0xcf, 0xcf, 0x56, 0x58,0x5a, 0x75,0x35, 0x25,0x45, 0x15,0x05);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb3, 0x14,0x00, 8);	
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==20) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(20, 0xdf, 0xdf, 0x56, 0x58,0x5a, 0x76,0x36, 0x26,0x46, 0x16,0x06);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb3, 0x15,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==21) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(21, 0xef,0xef, 0x56, 0x58,0x5a, 0x77,0x37, 0x27,0x47, 0x17,0x07);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb3, 0x17,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==22) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(22, 0xff,0xff, 0x56, 0x58,0x5a, 0x78,0x38, 0x28,0x48, 0x18,0x08);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb4, 0x11,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==23) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(23, 0xaf, 0xff,0x50, 0x51,0x52, 0x79,0x39, 0x29,0x49, 0x19,0x09);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb4, 0x12,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==24) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(24, 0xaf, 0xff,0x53, 0x54,0x57, 0x7a,0x3a, 0x2a,0x4a, 0x1a,0x0a);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb4, 0x14,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==25) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(25, 0xaf, 0xff,0x59, 0x5b,0x5c, 0x7b,0x3b, 0x2b,0x4b, 0x1b,0x0b);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb4, 0x15,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==26) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(26, 0xaf,0xff, 0x5d, 0x5e,0x5f, 0x7c,0x3c, 0x2c,0x4c, 0x1c,0x0c);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb4, 0x17,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==27) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(27, 0xbf,0xff, 0x50, 0x51,0x52, 0x7d,0x3d, 0x2d,0x4d, 0x1d,0x0d);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb5, 0x11,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==28) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(28, 0xbf,0xff, 0x53, 0x54,0x57, 0x7e,0x3e, 0x2e,0x4e, 0x1e,0x0e);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb5, 0x12,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==29) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(29, 0xbf,0xff, 0x59, 0x5b,0x5c, 0x7f,0x3f, 0x2f,0x4f, 0x1f,0x0f);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb5, 0x14,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==30) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(30, 0xbf,0xff, 0x5d, 0x5e,0x5f, 0x67,0x6a, 0x6d,0x5c, 0x11,0x15);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb5, 0x15,0x08, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==31) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(31, 0xcf,0xff, 0x50, 0x51,0x52, 0x68,0x6b, 0x6e,0x5d, 0x13,0x16);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb5, 0x17,0x00, 8);
                                                                                                                                  }
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==32) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  SlaveTurnOnOFF(32, 0xcf,0xff, 0x53, 0x54,0x57, 0x69,0x6c, 0x6f,0x5e, 0x14,0x17);
                                                                                                                                  LCD_Display(Bmp_sXuanze, 0xb6, 0x11,0x00, 8);
                                                                                                                                  }

                                                                                                                          
                                                                          if((g_Key2StateCheck==1)&&(g_ChooseNumber5==33) ){g_BackCounter1=0;
                                                                                                                                  g_Key2StateCheck=0;
                                                                                                                                  g_ChooseNumber5=2;
                                                                                                                                  break;
                                                                                                                                  }
                                                          }
                                                          
                                                          Display2_2Page();
                                                          XuanZeDisplay(0xb6, 0xb7,0x15,0x03);
                                                        }
                                                if((g_Key2StateCheck==1)&&(g_ChooseNumber4==5)) {g_BackCounter1=0;
                                                                                                        g_Key2StateCheck=0;
                                                                                                        g_ChooseNumber4=0;
                                                                                                        break;	}
                                        }
                                Display1Page();
                              
                                XuanZeDisplay(0xb4, 0xb5,0x16,0x08);
                        }
                if((g_Key2StateCheck==1)&&(g_DisplayNumber==2)) 
                {	g_BackCounter1=0;
                        PhaseABC=2;
                        g_Key2StateCheck=0;
                        DisableKey();
                        LCD_Clear();
                        LED_ParameterName();
                        LCD_Display(Bmp_Phase, 0xb0, 0x13,0x0a, 16);
                        LCD_Display(Bmp_Phase+16, 0xb1, 0x13,0x0a, 16);
                        LCD_Display(Bmp_A, 0xb0, 0x13,0x02, 8);
                        LCD_Display(Bmp_A+8, 0xb1, 0x13,0x02, 8);
                        if(g_SepCom==0)
                        PhaseABC=0;
                        LCD_ReDisplay(PhaseABC);
                        Delay(0x0007ffff);
                        
                        LCD_Display(Bmp_B, 0xb0, 0x13,0x02, 8);
                        LCD_Display(Bmp_B+8, 0xb1, 0x13,0x02, 8);
                        
                        if(g_SepCom==0)
                          PhaseABC=1;
                          LCD_ReDisplay(PhaseABC);
                        Delay(0x0007ffff);
                        
                        LCD_Display(Bmp_C, 0xb0, 0x13,0x02, 8);
                        LCD_Display(Bmp_C+8, 0xb1, 0x13,0x02, 8);
                        PhaseABC=2;
                       
                        LCD_ReDisplay(PhaseABC);
                        Delay(0x0007ffff);
                        PhaseABC=2;
                        Display1Page();
                        XuanZeDisplay(0xb6, 0xb7,0x13,0x00);
                        EnableKey();
                }
                if((g_Key2StateCheck==1)&&(g_DisplayNumber==3)) 
                {	g_BackCounter1=0;
                        PhaseABC=0;
                        g_Key2StateCheck=0;
                        DisableKey();
                        if((g_Slave00a==0)&&(g_Slave00b==0)&&(g_Slave00c==0))
                           RUN_LED_OFF;//GPIOC1 运行指示灯
                        if(g_Online==0) {g_Online=1;}
                        else  {g_Online=0;}
                        Display1Page();
                        XuanZeDisplay(0xb6, 0xb7,0x16,0x08);
                        EnableKey();
        
                }
                Manu2AutoChange();
                if(g_AutoManual==0) {break;}
                }
		}
        /* 自动运行*/
  	if(g_AutoManual==0) 
        {
          
          if((g_MasterSlave==1)&&(g_CloseState==0))
          {
            ResetTouchScreen();
            IWDG_Conf();//开硬狗
          }
          g_CloseState=1;//进入运行状态
          DisableCap();
          enableInterrupts();
          uint8_t PhaseABC=0;
          
          if((g_MasterSlave!=1)&&(g_IPAdress!=1))//表示从机
          {	
              uint8_t ReceiveNum=0;//20个循环内收到通讯表示正常
              uint8_t ReceiveFirst=0;//第一次通讯后标志位
             
              DisableCap();  //9-19
              enableInterrupts();
              UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);//使能接收中断
              UART1->CR2 |= (uint8_t)(UART1_CR2_REN); //使能接收
              if(1==Add_Alloc_Done)
                 UART1->CR2 |= (uint8_t)(UART1_CR2_RWU); //使能RWU
              UART1->CR2 &= (uint8_t)(~UART1_CR2_TEN); //发送被 禁止*/
              LCD_Clear();
              GPIOA->ODR &= (uint8_t)(~GPIO_PIN_3);
              IWDG_Conf();//开硬狗
                  
              while(1)//等待通讯中断
              {	
                PhaseABC=2;
                LCD_Clear();
                LED_ParameterName();
                LCD_Display(Bmp_Phase, 0xb0, 0x13,0x0a, 16);
                LCD_Display(Bmp_Phase+16, 0xb1, 0x13,0x0a, 16);
                LCD_Display(Bmp_A, 0xb0, 0x13,0x02, 8);
                LCD_Display(Bmp_A+8, 0xb1, 0x13,0x02, 8);
                
                if(g_SepCom==0)
                    PhaseABC=0;
                   // IWDG_ReloadCounter();// 喂狗
                g_Protect=LCD_ReDisplay(PhaseABC);
                IWDG_ReloadCounter();// 喂狗
                if(0==IsMasterAlive)
                {                   
                   if((g_PowerFactor<g_PFTou)&&(g_ReactivePowerValue>g_ReactiveQie)&&(((g_CurrentValue*10)>(g_CurrentOnOffValue*g_CTPhaseA)))&&(PF_Var_Neg==0)&&(g_Slave00a==0))  
                      {	
                        DisableKey();
                        EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
                        EnableCapAS();
                      }
                   else  if(((g_PowerFactor>g_PFQie)||(PF_Var_Neg==1)))   
                      {	
                        if((g_SepCom==0)&&(g_Slave00a==1))
                        {
                          DisableKey();
                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                          EnableCapA();
                        }
                      }
                }
                Delay(0x00015fff);
                if(g_SepCom==0)
                    PhaseABC=1;
                    
                LCD_Display(Bmp_B, 0xb0, 0x13,0x02, 8);
                LCD_Display(Bmp_B+8, 0xb1, 0x13,0x02, 8);
                
                g_Protect=LCD_ReDisplay(PhaseABC);
                IWDG_ReloadCounter();// 喂狗
                if(0==IsMasterAlive)
                {
                  if((g_PowerFactor<g_PFTou)&&(g_ReactivePowerValue>g_ReactiveQie)&&(((g_CurrentValue*10)>(g_CurrentOnOffValue*g_CTPhaseA)))&&(PF_Var_Neg==0)&&(g_Slave00b==0))  
                    {
                      DisableKey();
                       EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
                       EnableCapBS();
                    }
                   else  if(((g_PowerFactor>g_PFQie)||(PF_Var_Neg==1))&&(g_Slave00b==1))   
                    {	
                      DisableKey();
                      EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                      EnableCapB();
                    }
                }
                    Delay(0x00015fff);
                if(g_SepCom==0)
                    PhaseABC=2;
                IWDG_ReloadCounter();
                LCD_Display(Bmp_C, 0xb0, 0x13,0x02, 8);
                LCD_Display(Bmp_C+8, 0xb1, 0x13,0x02, 8);
                
                g_Protect=LCD_ReDisplay(PhaseABC);
                IWDG_ReloadCounter();// 喂狗
                if(0==IsMasterAlive)
                {
                  if((g_PowerFactor<g_PFTou)&&(g_ReactivePowerValue>g_ReactiveQie)&&(((g_CurrentValue*10)>(g_CurrentOnOffValue*g_CTPhaseA)))&&(PF_Var_Neg==0)&&(g_Slave00c==0))  
                    {	
                      DisableKey();
                      EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
                      EnableCapCS();
                    }
                   else  if((g_PowerFactor>g_PFQie)||(PF_Var_Neg==1))   
                    {	
                      if((g_SepCom==0)&&(g_Slave00c==1))
                          {
                            DisableKey();
                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                            EnableCapC();
                          }
                      if((g_SepCom==1)&&(g_Slave00a==1))
                          {
                            DisableKey();
                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                            EnableCapC();
                          }
                    }
                }
                Delay(0x00015fff);
                IWDG_ReloadCounter();// 喂狗
                Display2_4Page();
                IWDG_ReloadCounter();// 喂狗
                Delay(0x00010fff);
                IWDG_ReloadCounter();// 喂狗
                PhaseABC=2;
                Display0Page();
                IWDG_ReloadCounter();// 喂狗
                Delay(Rand_Volnum*1000);
                IWDG_ReloadCounter();// 喂狗
                 //   LCD_Clear();1-5
                TIM4_Cmd(ENABLE);      //从机状态启动TIM4，等待主机轮训时关闭，超时则从机自动运行
                ERR_LED_OFF;
                if(!GPIO_ReadInputPin(KEY_PORTE, KEY_7))
                {	Delay(0x00ff);	// 消抖
                        if(!GPIO_ReadInputPin(KEY_PORTE, KEY_7))
                        {	//ReceiveNum=0;
                                g_SlaveOK=0;
                                g_AutoManual=1;
                                break;
                        }
                }
                if((g_SlaveOK==1)&&(ReceiveFirst==0))//第一次通讯启动后开始计数判断是否主机故障 
                   ReceiveFirst=1;
                        
                if((g_SlaveOK==1)&&(ReceiveFirst==1))
                   ReceiveNum=ReceiveNum+1;
                if((ReceiveNum==10))
                   ReceiveNum=0;                        
              }
                         
          }
          if((g_MasterSlave==1)&&(g_IPAdress==1))//表示主机
            {
               //A相
              Auto2ManuChange();
              if(g_AutoManual==0)
              {
                Display0Page();
                Delay(0x000ffff);
                 Display2_4Page();
                Delay(0x000ffff);
                IWDG_ReloadCounter();// 喂狗
                LCD_Clear();						
                LED_ParameterName();
                LCD_Display(Bmp_Phase, 0xb0, 0x13,0x0a, 16);
                LCD_Display(Bmp_Phase+16, 0xb1, 0x13,0x0a, 16);
                LCD_Display(Bmp_A, 0xb0, 0x13,0x02, 8);
                LCD_Display(Bmp_A+8, 0xb1, 0x13,0x02, 8);
                if(g_SepCom==0)
                  g_Protect=LCD_ReDisplay(PhaseABC);
                else
                  g_Protect=LCD_ReDisplay(2);
                IWDG_ReloadCounter();// 喂狗
                  if((g_PowerFactor<g_PFTou)&&(g_ReactivePowerValue>g_ReactiveQie)&&(((g_CurrentValue*10)>(g_CurrentOnOffValue*g_CTPhaseA)))&&(PF_Var_Neg==0)&&(g_TotalOnNumber1!=g_EquipmentNumber))   //ltd135
                  {	
                    if(g_TotalOnNumber1<g_EquipmentNumber)
                      {	
                        g_OnNum1=g_OnNum1+1;
                        g_TotalOnNumber1=g_TotalOnNumber1+1;
                        if(g_OnNum1>g_EquipmentNumber)  {g_OnNum1=1;}
                
                        if(g_OnNum1==1)
                        {	
                          DisableKey();
                          EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
                          //g_SwitchON=1;
                          EnableCapAS();
                          
                          Delay(0x00008fff);
                          IWDG_ReloadCounter();// 喂狗
                          RS485_SendState(1, 1);
                          Delay(0x00000fff);
  
                        }
                        switch(g_OnNum1)
                        {
                          case 2:SwitchOnOff(0x82, 0x61);//给地址002 的机子发投切信息
                            break;
                          case 3:SwitchOnOff(0x83, 0x61);//给地址003 的机子发投切信息
                          break;
                          case 4:SwitchOnOff(0x84, 0x61);//给地址004 的机子发投切信息
                            break;
                          case 5:SwitchOnOff(0x85, 0x61);//给地址005 的机子发投切信息
                            break;
                          case 6:SwitchOnOff(0x86, 0x61);//给地址006 的机子发投切信息
                            break;
                          case 7:SwitchOnOff(0x87, 0x61);//给地址007 的机子发投切信息
                            break;
                          case 8:SwitchOnOff(0x88, 0x61);//给地址008 的机子发投切信息
                            break;
                          case 9:SwitchOnOff(0x89, 0x61);//给地址009 的机子发投切信息
                            break;
                          case 10:SwitchOnOff(0x8a, 0x61);//给地址010 的机子发投切信息
                            break;
                          case 11:SwitchOnOff(0x8b, 0x61);//给地址011 的机子发投切信息
                            break;
                          case 12:SwitchOnOff(0x8c, 0x61);//给地址012 的机子发投切信息
                            break;
                          case 13:SwitchOnOff(0x8d, 0x61);//给地址013 的机子发投切信息
                            break;
                          case 14:SwitchOnOff(0x8e, 0x61);//给地址014 的机子发投切信息
                            break;
                          case 15:SwitchOnOff(0x8f, 0x71);//给地址015 的机子发投切信息
                            break;
                          case 16:SwitchOnOff(0x9f, 0x72);//给地址016 的机子发投切信息
                            break;
                          case 17:SwitchOnOff(0xaf, 0x73);//给地址017 的机子发投切信息
                            break;
                          case 18:SwitchOnOff(0xbf, 0x74);//给地址018 的机子发投切信息
                            break;
                          case 19:SwitchOnOff(0xcf, 0x75);//给地址019 的机子发投切信息
                            break;
                          case 20:SwitchOnOff(0xdf, 0x76);//给地址020 的机子发投切信息
                            break;
                          case 21:SwitchOnOff(0xef, 0x77);//给地址021 的机子发投切信息
                            break;
                          case 22:SwitchOnOff(0xff, 0x78);//给地址022 的机子发投切信息
                            break;
                          case 23:SwitchOnOff(0xff, 0x79);//给地址023 的机子发投切信息
                            break;
                          case 24:SwitchOnOff(0xff, 0x7a);//给地址024 的机子发投切信息
                            break;
                          case 25:SwitchOnOff(0xff, 0x7b);//给地址025 的机子发投切信息
                            break;
                         default :
                            break;
                        }
                        Delay(0x000ffff);
                        IWDG_ReloadCounter();// 喂狗
                        DisableCap();
                        EnableKey();
                        SwitchOver=1;
                      
                      }
                  }
                  if(((g_PowerFactor>g_PFQie)||(PF_Var_Neg==1))&&(g_TotalOnNumber1!=0))   //ltd135
                  {	
                    g_OffNum1=g_OffNum1+1;
                    g_TotalOnNumber1=g_TotalOnNumber1-1;
                    if(g_OffNum1>g_EquipmentNumber) {g_OffNum1=1;}
            
                    if(g_OffNum1==1)
                                            {	DisableKey();
                                                    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                                           
                                                    if(g_SepCom==0)
                                                    EnableCapA();
                                                  
                                                    Delay(0x00008fff);
                                                    RS485_SendState(1, 2);
                                                    IWDG_ReloadCounter();// 喂狗
                                                    Delay(0x00000fff);
                                                   
                                            }
                     switch(g_OffNum1)
                        {
                          case 2:SwitchOnOff(0x82, 0x62);//给地址002 的机子发投切信息
                            break;
                          case 3:SwitchOnOff(0x83, 0x62);//给地址003 的机子发投切信息
                          break;
                          case 4:SwitchOnOff(0x84, 0x62);//给地址004 的机子发投切信息
                            break;
                          case 5:SwitchOnOff(0x85, 0x62);//给地址005 的机子发投切信息
                            break;
                          case 6:SwitchOnOff(0x86, 0x62);//给地址006 的机子发投切信息
                            break;
                          case 7:SwitchOnOff(0x87, 0x62);//给地址007 的机子发投切信息
                            break;
                          case 8:SwitchOnOff(0x88, 0x62);//给地址008 的机子发投切信息
                            break;
                          case 9:SwitchOnOff(0x89, 0x62);//给地址009 的机子发投切信息
                            break;
                          case 10:SwitchOnOff(0x8a, 0x62);//给地址010 的机子发投切信息
                            break;
                          case 11:SwitchOnOff(0x8b, 0x62);//给地址011 的机子发投切信息
                            break;
                          case 12:SwitchOnOff(0x8c, 0x62);//给地址012 的机子发投切信息
                            break;
                          case 13:SwitchOnOff(0x8d, 0x62);//给地址013 的机子发投切信息
                            break;
                          case 14:SwitchOnOff(0x8e, 0x62);//给地址014 的机子发投切信息
                            break;
                          case 15:SwitchOnOff(0x8f, 0x31);//给地址015 的机子发投切信息
                            break;
                          case 16:SwitchOnOff(0x9f, 0x32);//给地址016 的机子发投切信息
                            break;
                          case 17:SwitchOnOff(0xaf, 0x33);//给地址017 的机子发投切信息
                            break;
                          case 18:SwitchOnOff(0xbf, 0x34);//给地址018 的机子发投切信息
                            break;
                          case 19:SwitchOnOff(0xcf, 0x35);//给地址019 的机子发投切信息
                            break;
                          case 20:SwitchOnOff(0xdf, 0x36);//给地址020 的机子发投切信息
                            break;
                          case 21:SwitchOnOff(0xef, 0x37);//给地址021 的机子发投切信息
                            break;
                          case 22:SwitchOnOff(0xff, 0x38);//给地址022 的机子发投切信息
                            break;
                          case 23:SwitchOnOff(0xff, 0x39);//给地址023 的机子发投切信息
                            break;
                          case 24:SwitchOnOff(0xff, 0x3a);//给地址024 的机子发投切信息
                            break;
                          case 25:SwitchOnOff(0xff, 0x3b);//给地址025 的机子发投切信息
                            
                         default :
                            break;
                        }
                            
              
                    Delay(0x000ffff);
                    IWDG_ReloadCounter();// 喂狗
                    DisableCap();
                    EnableKey();
                    SwitchOver=1;
                          
                  }
              }
                      //B相
              Auto2ManuChange();
              if(g_AutoManual==0)
              {	
                DisableCap();
                EnableKey();
                Delay(0x000ffff);

                LCD_Display(Bmp_B, 0xb0, 0x13,0x02, 8);
                LCD_Display(Bmp_B+8, 0xb1, 0x13,0x02, 8);
                PhaseABC=1;
                if(g_SepCom==0)
                   g_Protect=LCD_ReDisplay(PhaseABC);
                else
                   g_Protect=LCD_ReDisplay(2);
                IWDG_ReloadCounter();// 喂狗
                if((g_PowerFactor<g_PFTou)&&(g_ReactivePowerValue>g_ReactiveQie)&&(((g_CurrentValue*10)>(g_CurrentOnOffValue*g_CTPhaseA)))&&(PF_Var_Neg==0)&&(g_TotalOnNumber2!=g_EquipmentNumber))  //ltd135
                {	
                  if(g_TotalOnNumber2<g_EquipmentNumber)
                    {	
                      g_OnNum2=g_OnNum2+1;
                      g_TotalOnNumber2=g_TotalOnNumber2+1;
                      if(g_OnNum2>g_EquipmentNumber)  {g_OnNum2=1;}
      
                      if(g_OnNum2==1)
                        {	
                          DisableKey();
                          if(g_SepCom==0)
                          {
                                  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
                        
                                  EnableCapBS();
                                  Delay(0x00008fff);
                                  RS485_SendState(1, 3);
                                  IWDG_ReloadCounter();// 喂狗
                                  Delay(0x00000fff);
                          }
                               
                        }
                       switch(g_OnNum2)
                        {
                        case 2:SwitchOnOff(0x82, 0x63);//给地址002 的机子发投切信息
                          break;
                        case 3:SwitchOnOff(0x83, 0x63);//给地址003 的机子发投切信息
                        break;
                        case 4:SwitchOnOff(0x84, 0x63);//给地址004 的机子发投切信息
                          break;
                        case 5:SwitchOnOff(0x85, 0x63);//给地址005 的机子发投切信息
                          break;
                        case 6:SwitchOnOff(0x86, 0x63);//给地址006 的机子发投切信息
                          break;
                        case 7:SwitchOnOff(0x87, 0x63);//给地址007 的机子发投切信息
                          break;
                        case 8:SwitchOnOff(0x88, 0x63);//给地址008 的机子发投切信息
                          break;
                        case 9:SwitchOnOff(0x89, 0x63);//给地址009 的机子发投切信息
                          break;
                        case 10:SwitchOnOff(0x8a, 0x63);//给地址010 的机子发投切信息
                          break;
                        case 11:SwitchOnOff(0x8b, 0x63);//给地址011 的机子发投切信息
                          break;
                        case 12:SwitchOnOff(0x8c, 0x63);//给地址012 的机子发投切信息
                          break;
                        case 13:SwitchOnOff(0x8d, 0x63);//给地址013 的机子发投切信息
                          break;
                        case 14:SwitchOnOff(0x8e, 0x63);//给地址014 的机子发投切信息
                          break;
                        case 15:SwitchOnOff(0x8f, 0x21);//给地址015 的机子发投切信息
                          break;
                        case 16:SwitchOnOff(0x9f, 0x22);//给地址016 的机子发投切信息
                          break;
                        case 17:SwitchOnOff(0xaf, 0x23);//给地址017 的机子发投切信息
                          break;
                        case 18:SwitchOnOff(0xbf, 0x24);//给地址018 的机子发投切信息
                          break;
                        case 19:SwitchOnOff(0xcf, 0x25);//给地址019 的机子发投切信息
                          break;
                        case 20:SwitchOnOff(0xdf, 0x26);//给地址020 的机子发投切信息
                          break;
                        case 21:SwitchOnOff(0xef, 0x27);//给地址021 的机子发投切信息
                          break;
                        case 22:SwitchOnOff(0xff, 0x28);//给地址022 的机子发投切信息
                          break;
                        case 23:SwitchOnOff(0xff, 0x29);//给地址023 的机子发投切信息
                          break;
                        case 24:SwitchOnOff(0xff, 0x2a);//给地址024 的机子发投切信息
                          break;
                        case 25:SwitchOnOff(0xff, 0x2b);//给地址025 的机子发投切信息
                          break;
                       default :
                          break;
                        }
                      Delay(0x0001ffff);
                      IWDG_ReloadCounter();// 喂狗
                      DisableCap();
                      
                      EnableKey();
                      SwitchOver=1;
                                        
                    }
                }
                if(((g_PowerFactor>g_PFQie)||(PF_Var_Neg==1))&&(g_TotalOnNumber2!=0))   //ltd135
                {	
                  g_OffNum2=g_OffNum2+1;
                  g_TotalOnNumber2=g_TotalOnNumber2-1;
                  if(g_OffNum2>g_EquipmentNumber) {g_OffNum2=1;}

                  if(g_OffNum2==1)
                    {	
                      if(g_SepCom==0)
                      {
                            DisableKey();
                            EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                
                            EnableCapB();
                            Delay(0x00008fff);
                            RS485_SendState(1, 4);
                            IWDG_ReloadCounter();// 喂狗
                            Delay(0x00000fff);
                      }
                           
                    }
                   switch(g_OffNum2)
                        {
                        case 2:SwitchOnOff(0x82, 0x64);//给地址002 的机子发投切信息
                          break;
                        case 3:SwitchOnOff(0x83, 0x64);//给地址003 的机子发投切信息
                        break;
                        case 4:SwitchOnOff(0x84, 0x64);//给地址004 的机子发投切信息
                          break;
                        case 5:SwitchOnOff(0x85, 0x64);//给地址005 的机子发投切信息
                          break;
                        case 6:SwitchOnOff(0x86, 0x64);//给地址006 的机子发投切信息
                          break;
                        case 7:SwitchOnOff(0x87, 0x64);//给地址007 的机子发投切信息
                          break;
                        case 8:SwitchOnOff(0x88, 0x64);//给地址008 的机子发投切信息
                          break;
                        case 9:SwitchOnOff(0x89, 0x64);//给地址009 的机子发投切信息
                          break;
                        case 10:SwitchOnOff(0x8a, 0x64);//给地址010 的机子发投切信息
                          break;
                        case 11:SwitchOnOff(0x8b, 0x64);//给地址011 的机子发投切信息
                          break;
                        case 12:SwitchOnOff(0x8c, 0x64);//给地址012 的机子发投切信息
                          break;
                        case 13:SwitchOnOff(0x8d, 0x64);//给地址013 的机子发投切信息
                          break;
                        case 14:SwitchOnOff(0x8e, 0x64);//给地址014 的机子发投切信息
                          break;
                        case 15:SwitchOnOff(0x8f, 0x41);//给地址015 的机子发投切信息
                          break;
                        case 16:SwitchOnOff(0x9f, 0x42);//给地址016 的机子发投切信息
                          break;
                        case 17:SwitchOnOff(0xaf, 0x43);//给地址017 的机子发投切信息
                          break;
                        case 18:SwitchOnOff(0xbf, 0x44);//给地址018 的机子发投切信息
                          break;
                        case 19:SwitchOnOff(0xcf, 0x45);//给地址019 的机子发投切信息
                          break;
                        case 20:SwitchOnOff(0xdf, 0x46);//给地址020 的机子发投切信息
                          break;
                        case 21:SwitchOnOff(0xef, 0x47);//给地址021 的机子发投切信息
                          break;
                        case 22:SwitchOnOff(0xff, 0x48);//给地址022 的机子发投切信息
                          break;
                        case 23:SwitchOnOff(0xff, 0x49);//给地址023 的机子发投切信息
                          break;
                        case 24:SwitchOnOff(0xff, 0x4a);//给地址024 的机子发投切信息
                          break;
                        case 25:SwitchOnOff(0xff, 0x4b);//给地址025 的机子发投切信息
                          break;
                       default :
                          break;
                        }
                  Delay(0x000ffff);
                  IWDG_ReloadCounter();// 喂狗
                  DisableCap();
                  EnableKey();
                  SwitchOver=1;
                        
                }
              }
                //C相
              Auto2ManuChange();
              if(g_AutoManual==0)
              { 
                DisableCap();
                EnableKey();
                Delay(0x000ffff);
                IWDG_ReloadCounter();// 喂狗
                LCD_Display(Bmp_C, 0xb0, 0x13,0x02, 8);
                LCD_Display(Bmp_C+8, 0xb1, 0x13,0x02, 8);
                PhaseABC=2;
                if(g_SepCom==0)
                    g_Protect=LCD_ReDisplay(PhaseABC);
                 else
                    g_Protect=LCD_ReDisplay(2);
                IWDG_ReloadCounter();// 喂狗
                if((g_PowerFactor<g_PFTou)&&(g_ReactivePowerValue>g_ReactiveQie)&&(((g_CurrentValue*10)>(g_CurrentOnOffValue*g_CTPhaseA)))&&(PF_Var_Neg==0)&&(g_TotalOnNumber3!=g_EquipmentNumber))  //ltd135
                {
                  if(g_TotalOnNumber3<g_EquipmentNumber)
                  {
                    g_OnNum3=g_OnNum3+1;
                    g_TotalOnNumber3=g_TotalOnNumber3+1;
                    if(g_OnNum3>g_EquipmentNumber)  {g_OnNum3=1;}
                                              
                    if(g_OnNum3==1)
                      {	
                        DisableKey();
                        EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
               
                        EnableCapCS();
                        Delay(0x00008fff);
                        RS485_SendState(1, 5);
                        IWDG_ReloadCounter();// 喂狗
                        Delay(0x00000fff);
                             
                      }
                     switch(g_OnNum3)
                        {
                        case 2:SwitchOnOff(0x82, 0x65);//给地址002 的机子发投切信息
                          break;
                        case 3:SwitchOnOff(0x83, 0x65);//给地址003 的机子发投切信息
                        break;
                        case 4:SwitchOnOff(0x84, 0x65);//给地址004 的机子发投切信息
                          break;
                        case 5:SwitchOnOff(0x85, 0x65);//给地址005 的机子发投切信息
                          break;
                        case 6:SwitchOnOff(0x86, 0x65);//给地址006 的机子发投切信息
                          break;
                        case 7:SwitchOnOff(0x87, 0x65);//给地址007 的机子发投切信息
                          break;
                        case 8:SwitchOnOff(0x88, 0x65);//给地址008 的机子发投切信息
                          break;
                        case 9:SwitchOnOff(0x89, 0x65);//给地址009 的机子发投切信息
                          break;
                        case 10:SwitchOnOff(0x8a, 0x65);//给地址010 的机子发投切信息
                          break;
                        case 11:SwitchOnOff(0x8b, 0x65);//给地址011 的机子发投切信息
                          break;
                        case 12:SwitchOnOff(0x8c, 0x65);//给地址012 的机子发投切信息
                          break;
                        case 13:SwitchOnOff(0x8d, 0x65);//给地址013 的机子发投切信息
                          break;
                        case 14:SwitchOnOff(0x8e, 0x65);//给地址014 的机子发投切信息
                          break;
                        case 15:SwitchOnOff(0x8f, 0x11);//给地址015 的机子发投切信息
                          break;
                        case 16:SwitchOnOff(0x9f, 0x12);//给地址016 的机子发投切信息
                          break;
                        case 17:SwitchOnOff(0xaf, 0x13);//给地址017 的机子发投切信息
                          break;
                        case 18:SwitchOnOff(0xbf, 0x14);//给地址018 的机子发投切信息
                          break;
                        case 19:SwitchOnOff(0xcf, 0x15);//给地址019 的机子发投切信息
                          break;
                        case 20:SwitchOnOff(0xdf, 0x16);//给地址020 的机子发投切信息
                          break;
                        case 21:SwitchOnOff(0xef, 0x17);//给地址021 的机子发投切信息
                          break;
                        case 22:SwitchOnOff(0xff, 0x18);//给地址022 的机子发投切信息
                          break;
                        case 23:SwitchOnOff(0xff, 0x19);//给地址023 的机子发投切信息
                          break;
                        case 24:SwitchOnOff(0xff, 0x1a);//给地址024 的机子发投切信息
                          break;
                        case 25:SwitchOnOff(0xff, 0x1b);//给地址025 的机子发投切信息
                          break;
                       default :
                          break;
                        }
                 
                    Delay(0x000ffff);
                    IWDG_ReloadCounter();// 喂狗
                    DisableCap();
                    
                    //disableInterrupts();
                    EnableKey();
                    SwitchOver=1;
                                                                
                  }
                }
                if(((g_PowerFactor>g_PFQie)||(PF_Var_Neg==1))&&(g_TotalOnNumber3!=0))  //ltd135
                {		
                  g_OffNum3=g_OffNum3+1;
                  g_TotalOnNumber3=g_TotalOnNumber3-1;
                  if(g_OffNum3>g_EquipmentNumber) {g_OffNum3=1;}
                  if(g_OffNum3==1)
                  {
                    DisableKey();
                    if(g_SepCom==0)
                    {
                      EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                      EnableCapC();
                    }
                    if(g_SepCom==1)
                    {
                      EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
                      EnableCapC();
                    }
                    
                    Delay(0x00008fff);
                    RS485_SendState(1, 6);
                    IWDG_ReloadCounter();// 喂狗
                    Delay(0x00000fff);
                          
                  }
                   switch(g_OffNum3)
                        {
                        case 2:SwitchOnOff(0x82, 0x66);//给地址002 的机子发投切信息
                          break;
                        case 3:SwitchOnOff(0x83, 0x66);//给地址003 的机子发投切信息
                        break;
                        case 4:SwitchOnOff(0x84, 0x66);//给地址004 的机子发投切信息
                          break;
                        case 5:SwitchOnOff(0x85, 0x66);//给地址005 的机子发投切信息
                          break;
                        case 6:SwitchOnOff(0x86, 0x66);//给地址006 的机子发投切信息
                          break;
                        case 7:SwitchOnOff(0x87, 0x66);//给地址007 的机子发投切信息
                          break;
                        case 8:SwitchOnOff(0x88, 0x66);//给地址008 的机子发投切信息
                          break;
                        case 9:SwitchOnOff(0x89, 0x66);//给地址009 的机子发投切信息
                          break;
                        case 10:SwitchOnOff(0x8a, 0x66);//给地址010 的机子发投切信息
                          break;
                        case 11:SwitchOnOff(0x8b, 0x66);//给地址011 的机子发投切信息
                          break;
                        case 12:SwitchOnOff(0x8c, 0x66);//给地址012 的机子发投切信息
                          break;
                        case 13:SwitchOnOff(0x8d, 0x66);//给地址013 的机子发投切信息
                          break;
                        case 14:SwitchOnOff(0x8e, 0x66);//给地址014 的机子发投切信息
                          break;
                        case 15:SwitchOnOff(0x8f, 0x01);//给地址015 的机子发投切信息
                          break;
                        case 16:SwitchOnOff(0x9f, 0x02);//给地址016 的机子发投切信息
                          break;
                        case 17:SwitchOnOff(0xaf, 0x03);//给地址017 的机子发投切信息
                          break;
                        case 18:SwitchOnOff(0xbf, 0x04);//给地址018 的机子发投切信息
                          break;
                        case 19:SwitchOnOff(0xcf, 0x05);//给地址019 的机子发投切信息
                          break;
                        case 20:SwitchOnOff(0xdf, 0x06);//给地址020 的机子发投切信息
                          break;
                        case 21:SwitchOnOff(0xef, 0x07);//给地址021 的机子发投切信息
                          break;
                        case 22:SwitchOnOff(0xff, 0x08);//给地址022 的机子发投切信息
                          break;
                        case 23:SwitchOnOff(0xff, 0x09);//给地址023 的机子发投切信息
                          break;
                        case 24:SwitchOnOff(0xff, 0x0a);//给地址024 的机子发投切信息
                          break;
                        case 25:SwitchOnOff(0xff, 0x0b);//给地址025 的机子发投切信息
                          break;
                       default :
                          break;
                        }
                  Delay(0x000ffff);
                  IWDG_ReloadCounter();// 喂狗
                  DisableCap();
                  EnableKey();
                  SwitchOver=1;
                        
                }
              }
             
              Auto2ManuChange();
              if(g_AutoManual==1)	
                PF_Var_Neg=0;
              DisableCap();
              EnableKey();
             
                        
            }
            Auto2ManuChange();
            if((g_IPAdress==1)&&(g_AutoManual==0))
            {	
              u8 CycleTest=2;
              u8 ErrorNumber=0;
                      
              if(g_Slave00a==0) 
              {RS485_SendState(1, 2);}// 主机A相切除状态
              if(g_Slave00a==1) 
              {RS485_SendState(1, 1);}// 主机A相投入状态
              if(g_Slave00b==0) 
              {RS485_SendState(1, 4);}// 主机B相切除状态
              if(g_Slave00b==1) 
              {RS485_SendState(1, 3);}// 主机B相投入状态
              if(g_Slave00c==0) 
              {RS485_SendState(1, 6);}// 主机C相切除状态
              if(g_Slave00c==1) 
              {RS485_SendState(1, 5);}// 主机C相投入状态
              IWDG_ReloadCounter();// 喂狗
              for(CycleTest=2;CycleTest<=g_EquipmentNumber;CycleTest++)
              {
		if(CycleTest==2)	
                {SlaveState(2, 0x82, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                  }//给地址002 的机子发确认信息
                if(CycleTest==3)	
                {SlaveState(3, 0x83, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址003 的机子发确认信息
                if(CycleTest==4)	
                {SlaveState(4, 0x84, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址004 的机子发确认信息
                if(CycleTest==5)	
                {SlaveState(5, 0x85, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址005 的机子发确认信息
                if(CycleTest==6)	
                {SlaveState(6, 0x86, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址006 的机子发确认信息
                if(CycleTest==7)	
                {SlaveState(7, 0x87, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址007 的机子发确认信息
                if(CycleTest==8)	
                {SlaveState(8, 0x88, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址008 的机子发确认信息
                if(CycleTest==9)	
                {SlaveState(9, 0x89, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                      {ErrorNumber=ErrorNumber+1;}
                   }//给地址009 的机子发确认信息
                if(CycleTest==10)	
                {SlaveState(10, 0x8a, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                      {ErrorNumber=ErrorNumber+1;}
                   }//给地址010 的机子发确认信息
                if(CycleTest==11)	
                {SlaveState(11, 0x8b, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址011 的机子发确认信息
                if(CycleTest==12)	
                {SlaveState(12, 0x8c, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址012 的机子发确认信息
                if(CycleTest==13)	
                {SlaveState(13, 0x8d, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址013 的机子发确认信息
                if(CycleTest==14)	
                {SlaveState(14, 0x8e, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址014 的机子发确认信息
                if(CycleTest==15)	
                {SlaveState(15, 0x8f, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址015 的机子发确认信息
                if(CycleTest==16)	
                {SlaveState(16, 0x9f, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址016 的机子发确认信息
                if(CycleTest==17)	
                {SlaveState(17, 0xaf, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址017 的机子发确认信息
                if(CycleTest==18)	
                {SlaveState(18, 0xbf, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址018 的机子发确认信息
                if(CycleTest==19)	
                {SlaveState(19, 0xcf, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址019 的机子发确认信息
                if(CycleTest==20)	
                {SlaveState(20, 0xdf, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址020 的机子发确认信息
                if(CycleTest==21)	
                {SlaveState(21, 0xef, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址021 的机子发确认信息
                if(CycleTest==22)	
                {SlaveState(22, 0xff, 0x56, 0x58, 0x5a);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址022 的机子发确认信息
                if(CycleTest==23)	
                {SlaveState(23, 0xaf, 0x50, 0x51, 0x52);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址023 的机子发确认信息
                if(CycleTest==24)	
                {SlaveState(24, 0xaf, 0x53, 0x54, 0x57);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址024 的机子发确认信息
                if(CycleTest==25)	
                {SlaveState(25, 0xaf, 0x59, 0x5b, 0x5c);
                   if((g_Slave0a==7)||(g_Slave0b==8)||(g_Slave0c==9))
                        {ErrorNumber=ErrorNumber+1;}
                   }//给地址025 的机子发确认信息
        
                IWDG_ReloadCounter();// 喂狗            
              }
                            
            }
                            
						
        if((g_MasterSlave==1)&&(g_IPAdress==1))//表示主机
                {
                  if(0==g_SepCom)
                  {
                    SendVoltage2Touch(g_VoltageValueSend1, 1);
                    SendVoltage2Touch(g_VoltageValueSend2, 2);
                    SendVoltage2Touch(g_VoltageValueSend3, 3);
                    SendPF2Touch(g_PowerFactorSend1, 1);
                    SendPF2Touch(g_PowerFactorSend2, 2);
                    SendPF2Touch(g_PowerFactorSend3, 3);
                    IWDG_ReloadCounter();// 喂狗
                  }
                else
                  {
                    SendVoltage2Touch(g_VoltageValueSend3, 1);
                    SendVoltage2Touch(g_VoltageValueSend3, 2);
                    SendVoltage2Touch(g_VoltageValueSend3, 3);
                    SendPF2Touch(g_PowerFactorSend3, 1);
                    SendPF2Touch(g_PowerFactorSend3, 2);
                    SendPF2Touch(g_PowerFactorSend3, 3);
                    IWDG_ReloadCounter();// 喂狗
                  }
                }                                                                               
        static	u8 k=0;
        for(k=0;k<=g_TotalCycleTimeNum;k++)
        {
          Delay(Rand_Volnum*10);
        }
	IWDG_ReloadCounter();// 喂狗
        }
  }
  
