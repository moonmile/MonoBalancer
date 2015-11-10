namespace MonoBalancer
open System
open System.Threading
open System.Diagnostics

type TickTimer( callback, state, dueTime, period ) as this =
  let mutable _cb:TimerCallback = callback
  let mutable _sw:Stopwatch = null
  let mutable _dueTime = dueTime
  let mutable _period  = period
  let mutable _loop = true
  let mutable _task:Thread = null

  do 
    _sw <- new Stopwatch()
    _task <- new Thread(this.onTimer)
    // _task.Start( state ) 

  new ( callback, period ) =
    TickTimer( callback, null, 0, period )

  member this.Start() =
    _dueTime <- 0 
    _task.Start( state )

  member this.Stop() =
    _loop <- false
  
  member this.onTimer() = 
    Thread.Sleep( _dueTime )
    _sw.Restart()
    while _loop do 
      let msec = _sw.ElapsedMilliseconds
      let rest = _period - int( msec % int64(_period))
      // 200msec だけ余らせてスリープ
      if ( rest > 200 ) then
        Thread.Sleep(rest - 200)
      while ( _sw.ElapsedMilliseconds < msec + int64(rest) ) do
        ()
      if ( _cb <> null ) then
        _cb.Invoke( state )
    _sw.Stop()
        
    






  
