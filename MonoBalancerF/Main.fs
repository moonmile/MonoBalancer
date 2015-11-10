open System
open System.Threading
open MonoBrickFirmware
open MonoBrickFirmware.Display.Dialogs
open MonoBrickFirmware.Display
open MonoBrickFirmware.Movement
open MonoBrickFirmware.Sensors
open MonoBrickFirmware.Sound
open System.Diagnostics
open MonoBalancer
open System.Threading.Tasks

type Balanc3R() =

    let SAMPLE_TIME = 20.0      // サンプリング間隔
    let WHEEL_DIAMETER = 42.0   // ホイールの直径

    let _dt = SAMPLE_TIME / 1000.0         // サンプリング間隔 sec単位
    let _radius = WHEEL_DIAMETER / 2000.0  // ホイールの直径 m単位
    let _gyro_type = 0  // ジャイロタイプ LEGO:0

    let motorL = new Motor(MotorPort.OutA)
    let motorR = new Motor(MotorPort.OutD)
    let speaker = new Speaker(0)
    let gyro = new EV3GyroSensor(SensorPort.In2)
    let ir = new EV3IRSensor(SensorPort.In4)

    let mutable _speed = 0.0
    let mutable _steering = 0.0

    // PID 制御の値
    let Kp = 0.6       // Kp
    let Ki = 14.0      // Ki
    let Kd = 0.005     // Kd
    // ゲイン
    let GAIN_ANGLE_VELOCITY = 1.3   // 角速度
    let GAIN_ANGLE = 25.0           // 角度
    let GAIN_WHEEL_SPEED = 75.0     // 車輪スピード 
    let GAIN_WHEEL_POSITION = 350.0 // 車輪位置

    let mutable _old_steering = 0.0
    let mutable _mean = 0.0
    let mutable _ang = 0.0
    let mutable _refpos = 0.0

    let mutable onStop:Action = null


    // ジャイロの加速度を取得する
    let GyroRate() =
        // LEGO のみ対応
        if _gyro_type = 0 then
            let mutable sum = 0.0
            for i=1 to 10 do
                sum <- sum + float(gyro.Read())
            sum / 10.0
        else
            raise( new ArgumentException("LEGO Gyro senser only"))


    // 位置を取得
    let Position() =
         _refpos <- _refpos + _dt * 0.002 
         _refpos

    let mutable _enc_index = 0
    let _enc_val : float array = Array.zeroCreate 8
    /// モーターの平均スピードを計算する
    /// 配列のサイズ * サンプリング間隔 分だけ以前のスピードと平均する
    let GetMotorSpeed() =
        _enc_index <- 
            if _enc_index + 1 >= _enc_val.Length then 0 else _enc_index + 1
        let compare_index = 
            if _enc_index + 1 >= _enc_val.Length then 0 else _enc_index + 1
        let ave = float((motorL.GetTachoCount() + motorR.GetTachoCount())/2)
        _enc_val.[_enc_index] <- ave
        let speed = ( _enc_val.[_enc_index] - _enc_val.[compare_index] )/( float(_enc_val.Length) * _dt ) 
        speed

    // ロボットの位置とスピードを計算する
    let ReadEncoders() =
        let speed = (_radius * GetMotorSpeed()) / 57.3
        let tc = float(motorL.GetTachoCount() + motorR.GetTachoCount()) / 2.0
        let position = (_radius * tc) / 57.3
        ( position, speed )


    let ReadGyro() =
        let curr_val = GyroRate()
        _mean <- (_mean * (1.0 - _dt * 0.2) + (curr_val * _dt * 0.2))
        let ang_vel = curr_val - _mean
        _ang <- _ang + _dt * ang_vel
        ( _ang, ang_vel )


    // 初期のジャイロを取得する
    let Calibrate() =
        // トーンを鳴らす
        speaker.PlayTone(uint16(440),uint16(100),10)
        Thread.Sleep(100)
        for i = 1 to 20 do
            _mean <- _mean + GyroRate()
            Thread.Sleep(5)     // 5msec待つ
        _mean <- _mean / 20.0
        Thread.Sleep(100)
        speaker.PlayTone(uint16(440),uint16(100),10)
        Thread.Sleep(100)
        speaker.PlayTone(uint16(440),uint16(100),10)
        _mean


    let CombineSensorValues( angle_velocity,
                                angle,
                                robotSpeed,
                                robotPosition,
                                motorPosition ) = 
        let c = GAIN_WHEEL_POSITION * (robotPosition - motorPosition)
        let d = GAIN_WHEEL_SPEED * robotSpeed
        let a = GAIN_ANGLE * angle
        let b = GAIN_ANGLE_VELOCITY * angle_velocity
        a + b + c + d

    let mutable acc_err = 0.0
    let mutable prev_err = 0.0
    // PID制御
    let PID( referenceVal, inputVal ) =
        let curr_err = inputVal - referenceVal 
        acc_err <- acc_err + curr_err * _dt
        let dif_err = (curr_err - prev_err ) / _dt 
        let output = Kp * curr_err + Ki * acc_err + Kd * dif_err 
        prev_err <- curr_err
        output

    /// モーターへのパワー（PWM）を計算して設定する
    let SetMotorPower( steering:float, avaragePower:float ) =
        let Limit(x,l,u) = 
            match x with
            | x when x < l -> l
            | x when x > u -> u
            | _ -> x
        let new_steering = Limit(steering,-50.0,50.0)
        let extra_pwr = 
            if new_steering = 0.0 then
                let sync_0 = 
                    if _old_steering <> 0.0 then
                        motorL.GetTachoCount() - motorR.GetTachoCount()
                    else
                        0
                float(motorL.GetTachoCount() - motorR.GetTachoCount() - sync_0) * 0.05
            else
                new_steering * -0.5
        let power_c = avaragePower - extra_pwr
        let power_b = avaragePower + extra_pwr
        _old_steering <- new_steering
        let pwrA = power_b * 0.021 / _radius
        let pwrD = power_c * 0.021 / _radius
        motorR.SetPower(sbyte(pwrA))
        motorL.SetPower(sbyte(pwrD))


    let mutable _nowOutOfBound = false
    let mutable _prevOutOfBound = false
    let mutable _outOfBoundCount = 0
    let Errors( averagePower:float ) =
        _nowOutOfBound <- Math.Abs( averagePower ) > 100.0
        if ( _nowOutOfBound = true && _prevOutOfBound = true ) then
            _outOfBoundCount <- _outOfBoundCount + 1
        else 
            _outOfBoundCount <- 0
        if _outOfBoundCount > 20 then
            Thread.Sleep(200)
            motorL.Off();
            motorR.Off();
            Lcd.Clear();
            let font = Font.MediumFont
            Lcd.WriteText(font, new MonoBrickFirmware.Display.Point(3, 5), "ERROR", false)
            speaker.PlayTone(uint16(800), uint16(100), 50)
            speaker.PlayTone(uint16(600), uint16(100), 50)
            speaker.PlayTone(uint16(300), uint16(100), 50)
            Thread.Sleep(4000)
            raise( new Exception("ERROR"))
        else
            _prevOutOfBound <- _nowOutOfBound
    

    do
        // センサーの初期値を設定
        gyro.Mode <- GyroMode.AngularVelocity
        ir.Mode <- IRMode.Remote
        // モーター状態をリセット
        motorL.Off()
        motorR.Off()
        motorL.ResetTacho()
        motorR.ResetTacho()
        // 0.1秒待ち
        Thread.Sleep(100)
        // 初期のジャイロ状態を取得
        _mean <- Calibrate()
        _speed <- 0.0
        _steering <- 0.0

    // バランスのためのループ
    member this.OnBalanceLoop( state:obj ) =
      try 
        let motorPosition = Position()
        let (position, speed) = ReadEncoders()
        let (angle, angleVelocity) = ReadGyro()
        let inputVal = CombineSensorValues(angleVelocity, angle, speed, position, motorPosition)
        let pid = PID( 0.0, inputVal )
        Errors( pid )
        SetMotorPower(_steering, pid)
      with 
        | _ -> 
          if ( onStop <> null ) then
            let font = Font.MediumFont
            Lcd.Update()
            Lcd.WriteText(font, new MonoBrickFirmware.Display.Point(3, 5), "ERROR", false)
            onStop.Invoke()

    // ループを停止させる
    member this.OnStop 
      with get() = onStop
      and set(v) = onStop <- v

    // 自動で動かす
    member this.MoveLoop( state:obj ) =
      let task = new Task( fun() -> 
        while true do
          _speed <- 0.0
          _steering <- 0.0
          Thread.Sleep(7000)
          speaker.PlayTone(uint16(440), uint16(50), 50)
          _speed <- 0.0
          _steering <- 20.0
          Thread.Sleep(7000)
          speaker.PlayTone(uint16(440), uint16(50), 50)
          _speed <- 0.0
          _steering <- -20.0
          Thread.Sleep(7000)
          speaker.PlayTone(uint16(440), uint16(50), 50)
      )
      task.Start()

    // 赤外線リモコンの受信ループ
    member this.RemoteLoop( state:obj ) =
      let task = new Task( fun() -> 
        while true do
          if ( ir.Channel = IRChannel.One ) then
            match ir.Read() with
            | 0 ->    // 何も押さない状態 
              _speed <- 0.0
              _steering <- 0.0
            | 3 ->    // 右上
              _speed <- 30.0
              _steering <- 15.0
            | 1 ->    // 左上
              _speed <- 30.0
              _steering <- -15.0
            | 4 ->    // 右下
              _speed <- -30.0
              _steering <- -15.0
            | 2 ->    // 左下
              _speed <- -30.0
              _steering <- 15.0
            | 5 ->    // 左上と右上
              _speed <- 40.0
              _steering <- 0.0
            | 8 ->    // 左下と右下
              _speed <- -40.0
              _steering <- 0.0
            | _ ->
              ()
      )
      task.Start()

let main() = 

    Console.WriteLine("MonoBalancer start...");
    let terminateProgram = new ManualResetEvent(false)    
    let ba = new Balanc3R()
    // ba.Initialize, ba.SetConstants は省略
    let tm = new TickTimer( new TimerCallback( ba.OnBalanceLoop ), 20 )
    tm.Start()
    Console.WriteLine("start to move")
    // バランスを崩したときに停止させる
    ba.OnStop <- new Action( fun () ->
        tm.Stop()
        Console.WriteLine("stop to move")
        terminateProgram.Set() |> ignore
      )
    // 自動で動くループ
    ba.MoveLoop(null)
    // 赤外線リモコンを使う
    // ba.RemoteLoop(null)
    // 無限ループ待ち
    terminateProgram.WaitOne() |> ignore     

main()

