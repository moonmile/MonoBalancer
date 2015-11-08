using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MonoBrickFirmware;
using MonoBrickFirmware.Movement;
using MonoBrickFirmware.Sensors;
using MonoBrickFirmware.Sound;
using System.Threading;
using MonoBrickFirmware.Display;
using System.Diagnostics;
namespace MonoBalanc3R
{
	class Program
	{

		static void Main(string[] args)
		{
			Console.WriteLine("MonoBlanc start...");

			var ba = new Balanc3R();
			TickTimer tm = new TickTimer(ba.OnBalanceLoop, 20);

			double GAIN_ANGLE_VELOCITY = 1.3;
			int GAIN_ANGLE = 25;
			int GAIN_WHEEL_SPEED = 75 ;
			int GAIN_WHEEL_POSITION = 350;

			ba.Initialize(
				0,  // LEGO Jyro 
				42, // wheel 42 mm
				22);// sample time 22msec
			ba.SetConstants(
				0.6,    // Kp
				14,     // Ki
				0.005,  // Kd
				GAIN_ANGLE_VELOCITY,    // Gain Angular Velocity 
				GAIN_ANGLE,     // Gain Angle    
				GAIN_WHEEL_SPEED,     // Gain Wheel speed 
				GAIN_WHEEL_POSITION);   // Gain Wheel position 

			// ba.BalanceLoop();
			// TickTimer tm = new TickTimer(ba.OnBalanceLoop, null, 0, 20);
			tm.Start ();
			Console.WriteLine("start to move");
			bool loop = true;
			ba.OnStop += () => { 
				tm.Stop();
				Console.WriteLine("stop to move");
				loop = false;
			};
			// 自動で動くループ
			// ba.MoveLoop(null);
			// 赤外線リモコンループ
			ba.RemoteLoop(null);
			// 無限ループ
			while (loop)
				Thread.Sleep(500);
		}

	}
	public class Balanc3R
	{
		private double _dt = 0.0;
		private double _radius = 0.0;
		private int _gyro_type = 0;

		Motor motorL;
		Motor motorR;
		Speaker speaker;
		EV3GyroSensor gyro;
		EV3IRSensor ir;


		double _speed = 0;
		double _steering = 0;

		double _kp;
		double _ki;
		double _kd;
		double _gainAngleVelocity;
		double _gainAngle;
		double _gainMotorSpeed;
		double _gainMotorPosition;

		double _old_steering = 0.0;

		/// <summary>
		/// コンストラクタ
		/// </summary>
		public Balanc3R()
		{
			motorR = new Motor(MotorPort.OutA); // 右足
			motorL = new Motor(MotorPort.OutD); // 左足
			speaker = new Speaker(0);
			gyro = new EV3GyroSensor(SensorPort.In2);
			// 角速度を取得
			gyro.Mode = GyroMode.AngularVelocity;
			ir = new EV3IRSensor(SensorPort.In4);
			ir.Mode = IRMode.Remote;

		}

		/// <summary>
		/// 初期化
		/// </summary>
		public void Initialize(int chooseSenser, int wheelDiameter, int sampleTime)
		{
			_dt = (sampleTime - 2) / 1000.0;    // 補正後 sec 単位へ
			_radius = wheelDiameter / 2000.0;   // 半径 m 単位へ
			_gyro_type = chooseSenser;          // ジャイロ種別 0:LEGO 1:HiTech 2:Dexter
			// Lモーターをリセット
			motorL.Off();
			motorR.Off ();
			motorL.ResetTacho();
			motorR.ResetTacho();
			// 内部変数を初期化
			_refpos = 0;
			// 0.1秒待ち
			Thread.Sleep(100);
			// 初期のジャイロを取得
			_mean = this.Calibrate();
			_speed = 0;
			_steering = 0;
			// _maxAcceleration = 0;
		}
		/// <summary>
		/// 初期値設定
		/// </summary>
		public void SetConstants(
			double kp,
			double ki,
			double kd,
			double angleVelocity,
			int angle,
			int wheelSpeed,
			int wheelPosition)
		{
			_kp = kp;
			_ki = ki;
			_kd = kd;
			_gainAngleVelocity = angleVelocity;
			_gainAngle = angle;
			_gainMotorSpeed = wheelSpeed;
			_gainMotorPosition = wheelPosition;
		}

		/// <summary>
		/// 初期のジャイロを取得する
		/// </summary>
		/// <returns></returns>
		public double Calibrate()
		{
			// トーンを鳴らす
			speaker.PlayTone(440, 100, 10);
			Thread.Sleep(100);
			_mean = 0;

			for (int i = 0; i < 20; i++)
			{
				_mean += GyroRate();
				Thread.Sleep(5); // 5msec 待つ
			}
			_mean = _mean / 20.0; // 平均する
			Thread.Sleep(100);  // 100msec 待つ
			speaker.PlayTone(440, 100, 10);
			Thread.Sleep(100);  // 100msec 待つ
			speaker.PlayTone(440, 100, 10);
			// 平均値を返す
			return _mean;
		}
		/// <summary>
		/// ジャイロで角速度を取得する
		/// </summary>
		/// <returns></returns>
		public int GyroRate()
		{
			// LEGO のみ可能
			if (_gyro_type == 0)
			{
				// 精度が悪いので10回の平均を出す
				int sum = 0;
				for ( int i=0; i<10; i++ )
				{
					sum += gyro.Read();
				}
				return sum / 10;
			}
			else
			{
				throw new ArgumentException("LEGO Gyro senser only");
			}
		}
		/// <summary>
		/// バランス用のループ
		/// </summary>
		/// <param name="state">State.</param>
		public void OnBalanceLoop( object state )
		{
			try
			{
				double motorPosition = Position();
				double robotPosition = 0.0;
				double robotSpeed = 0.0;
				ReadEncoders(out robotPosition, out robotSpeed);
				double angle = 0.0;
				double angleVelocity = 0.0;
				ReadGyro(out angle, out angleVelocity);
				double inputVal = CombineSensorValues(angleVelocity, angle, robotSpeed, robotPosition, motorPosition);
				double pid = PID(_kp, _ki, _kd, _dt, 0.0, inputVal);
				Errors(pid); // エラーの場合は例外が発生する
				SetMotorPower(_steering, pid);
			}
			catch
			{
				// 停止する
				if (OnStop != null)
				{
					Font font = Font.MediumFont;
					Lcd.Update();
					Lcd.WriteText(font, new MonoBrickFirmware.Display.Point(3, 5), "ERROR", false);
					OnStop();
				}
			}
		}
		// ループを停止させるとき
		public event Action OnStop;
		public void MoveLoop(object state)
		{
			Task task = new Task(() =>
				{
					while (true)
					{
						_speed = 0;
						_steering = 0;
						Thread.Sleep(7000);
						speaker.PlayTone(440, 50, 50);
						_speed = 0;
						_steering = 20;
						Thread.Sleep(7000);
						speaker.PlayTone(440, 50, 50);
						_speed = 0;
						_steering = -20;
						Thread.Sleep(7000);
						speaker.PlayTone(440, 50, 50);
					}
				});
			task.Start();
		}

		/// <summary>
		/// 赤外線リモコン用のループ
		/// </summary>
		/// <param name="state"></param>
		public void RemoteLoop(object state)
		{
			Task task = new Task(() =>
				{
					while (true)
					{
						if (ir.Channel == IRChannel.One)
						{
							switch (ir.Read())
							{
							case 0: // 何も押さない
								_speed = 0;
								_steering = 0;
								break;
							case 3: // 右上
								_speed = 30;
								_steering = 15;
								break;
							case 1: // 左上
								_speed = 30;
								_steering = -15;
								break;
							case 4: // 右下
								_speed = -30;
								_steering = -15;
								break;
							case 2: // 左下
								_speed = -30;
								_steering = 15;
								break;
							case 5: // 左上と右上
								_speed = 40;
								_steering = 0;
								break;
							case 8: // 左下と右下
								_speed = -40;
								_steering = 0;
								break;
							}
						}
						Thread.Sleep(200);
					}
				});
			task.Start();
		}


		double _refpos = 0;
		/// <summary>
		/// 位置を取得
		/// </summary>
		/// <returns></returns>
		public double Position()
		{
			_refpos += _dt * _speed * 0.002;    // 0.0022sec * 速度 * 係数？
			// 静止状態のときは spped が 0.0
			return _refpos;
		}
		/// <summary>
		/// ロボットの位置とスピードを計算する
		/// </summary>
		/// <returns></returns>
		public void ReadEncoders( out double position, out double speed )
		{
			double motorSpeed = GetMotorSpeed();
			speed = (_radius * motorSpeed) / 57.3;      // 定数？
			double tc = (motorL.GetTachoCount() + motorR.GetTachoCount()) / 2.0;
			position = (_radius * tc) / 57.3;
		}
		/// <summary>
		/// ジャイロから読み込み
		/// </summary>
		/// <param name="angle"></param>
		/// <param name="angleVelocity"></param>
		double _mean = 0;
		double _ang = 0;
		public void ReadGyro( out double angle, out double angleVelocity )
		{
			double curr_val = GyroRate();
			_mean = (_mean * (1.0 - _dt * 0.2) + (curr_val * _dt * 0.2));
			double ang_vel = curr_val - _mean;
			_ang += _dt * ang_vel;
			// 戻り値
			angle = _ang;
			angleVelocity = ang_vel;
		}

		/// <summary>
		/// モーターの平均スピードを計算する
		/// 配列のサイズ * サンプリング間隔 分だけ以前のスピードと平均する
		/// </summary>
		/// <returns></returns>
		int _enc_index = 0;
		double[] _enc_val = new double[8];
		public double GetMotorSpeed()
		{
			_enc_index++;
			if (_enc_index >= _enc_val.Length) _enc_index = 0;
			int compare_index = _enc_index + 1;
			if (compare_index >= _enc_val.Length) compare_index = 0;
			int ave = (motorL.GetTachoCount() + motorR.GetTachoCount()) / 2;
			_enc_val[_enc_index] = ave;
			// 平均スピードを計算する
			double speed = (_enc_val[_enc_index] - _enc_val[compare_index]) / (_enc_val.Length * _dt);
			return speed;
		}
		/// <summary>
		/// PID制御への入力値を計算する（重みづけ）
		/// </summary>
		/// <param name="angle_velocity"></param>
		/// <param name="angle"></param>
		/// <param name="robotSpeed"></param>
		/// <param name="robotPosition"></param>
		/// <param name="motorPosition"></param>
		/// <returns></returns>
		public double CombineSensorValues(
			double angle_velocity,
			double angle,
			double robotSpeed,
			double robotPosition,
			double motorPosition)
		{
			double c = _gainMotorPosition * (robotPosition - motorPosition);
			double d = _gainMotorSpeed * robotSpeed;
			double a = _gainAngle * angle;
			double b = _gainAngleVelocity * angle_velocity;
			double w = a + b + c + d;
			return w;
		}

		/// <summary>
		/// PID制御
		/// </summary>
		/// <param name="kp"></param>
		/// <param name="ki"></param>
		/// <param name="kd"></param>
		/// <param name="sampleTime"></param>
		/// <param name="referenceVal"></param>
		/// <param name="inputVal"></param>
		/// <returns></returns>
		/// curr_err : 現在のずれ(P)
		/// acc_err :  積算のずれ(I)
		/// prev_err:  差分のずれ(D)
		/// referance value が 0 の場合は倒立。入力値をできるだけ0に近づける。
		double acc_err = 0;
		double prev_err = 0;
		public double PID(
			double kp,
			double ki,
			double kd,
			double sampleTime,
			double referenceVal,
			double inputVal)
		{
			double curr_err = inputVal - referenceVal;
			acc_err = acc_err + curr_err * _dt;
			double dif_err = (curr_err - prev_err) / _dt;
			double c = kp * curr_err;
			double b = ki * acc_err;
			double a = kd * dif_err;
			double output = a + b + c;
			prev_err = curr_err;
			return output;
		}
		/// <summary>
		/// モーターへのパワー（PWM）を計算する
		/// motorR : portA
		/// motorL : portD
		/// </summary>
		/// <param name="steering"></param>
		/// <param name="avaragePower"></param>
		public void SetMotorPower(double steering, double avaragePower)
		{
			double new_steering = Limit(steering, -50, 50);
			double extra_pwr = 0.0;
			if (new_steering == 0)
			{
				double sync_0 = 0.0;
				if (_old_steering != 0)
				{
					sync_0 = motorL.GetTachoCount() - motorR.GetTachoCount();
				}
				extra_pwr = (motorL.GetTachoCount() - motorR.GetTachoCount() - sync_0) * 0.05;
			}
			else
			{
				extra_pwr = new_steering * -0.5;
			}
			double power_c = avaragePower - extra_pwr;
			double power_b = avaragePower + extra_pwr;
			_old_steering = new_steering;

			double pwrA = power_b * 0.021 / _radius;
			double pwrD = power_c * 0.021 / _radius;
			motorR.SetPower((sbyte)pwrA);
			motorL.SetPower((sbyte)pwrD);
		}

		public double Limit(double x, double lower, double upper)
		{
			if (x < lower) return lower;
			if (x > upper) return upper;
			return x;
		}

		bool _nowOutOfBound = false;
		bool _prevOutOfBound = false;
		double _outOfBoundCount = 0;
		public void Errors(double averagePower)
		{
			_nowOutOfBound = Math.Abs(averagePower) > 100.0;
			if (_nowOutOfBound & _prevOutOfBound)
			{
				_outOfBoundCount++;
			}
			else
			{
				_outOfBoundCount = 0;
			}
			if (_outOfBoundCount > 20)
			{
				Thread.Sleep(100);
				motorL.Off();
				motorR.Off();
				Lcd.Clear();
				Font font = Font.MediumFont;
				Lcd.WriteText(font, new MonoBrickFirmware.Display.Point(3, 5), "ERROR", false);
				speaker.PlayTone(800, 100, 50);
				speaker.PlayTone(600, 100, 50);
				speaker.PlayTone(300, 100, 50);
				Thread.Sleep(4000);
				throw new Exception("ERROR");
			}
			else
			{
				_prevOutOfBound = _nowOutOfBound;
			}
		}
	}

	public class TickTimer 
	{
		TimerCallback _cb;
		Stopwatch _sw;
		int _dueTime;
		int _period;
		bool _loop = true;
		Thread _task;

		public TickTimer(TimerCallback callback, object state, int dueTime, int period)
		{
			_cb = callback;
			_dueTime = dueTime;
			_period = period;
			_sw = new Stopwatch();
			_task = new Thread(onTimer);
			_task.Start( state );
		}
		public TickTimer(TimerCallback callback, int period)
		{
			_cb = callback;
			_period = period;
			_sw = new Stopwatch();
			_task = new Thread(onTimer);
		}
		public void Start( object state = null )
		{
			_dueTime = 0;
			_task.Start( state );
		}

		public void Stop()
		{
			_loop = false;
		}
		private void onTimer(object state)
		{
			Thread.Sleep(_dueTime);
			_sw.Restart();
			while (_loop)
			{
				long msec = _sw.ElapsedMilliseconds;
				int rest = _period - (int)(msec % _period);
				// 200msecだけ余らせてスリープ
				if (rest > 200)
				{
					Thread.Sleep(rest - 200);
				}
				// 200msecの間、ちょうどになるまでループで待つ
				while (true)
				{
					if (_sw.ElapsedMilliseconds >= msec + rest)
					{
						break;
					}
				}
				if (_cb != null)
				{
					_cb(state);
				}
			}
			_sw.Stop();
		}
	}
}
