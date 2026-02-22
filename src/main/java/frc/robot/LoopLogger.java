
/* //////////////////////////////////////////////////////////////////////////

LoopLogger
from LASA Robotics

LoopLogger is a lightweight module for monitoring execution loops times
for ensuring appropriate execution for a realtime system.

The primary output is the "Time Slot" table which shows the number of
times each period loop took to execute per a period of time along with
start time jitter.  The output is sent to the console each time the
robot is disabled.

&&&&&&& TODO show example output and decode &&&&

Recommended usage to monitor the output of LoopLogger across the season
to catch if and when any extraneous code exeuction time materializes.

Additional statistics are reported for average time spent per loop,
memory allocation on the heap, and occurances of garbage collections.

This module is very lightweight consuming on average less than 8us
per cycle with no dynamic allocation of memory on the heap.

To implement in your code:

(1) Add the LoopLogger object to your Robot class:

  public class Robot TimedRobot {

    private final LoopLogger ll= new LoopLogger();   <<< ADD THIS LINE

    ...
  }

(2) Call the start and end methods at the beginning and end for each 
    of the periodic methods:

  public void robotPeriodic() {
    ll.RobotStart();            <<< ADD THIS LINE, BEFORE super.robotPeriod()
    super.robotPeriodic(); 
    ...
    ...  your code here
    ...
    ll.RobotEnd( isEnabled() ); <<< ADD THIS LINE
  }

  public void autonomousPeriodic() {
    ll.AutonomousStart();       <<< ADD THIS LINE
    ...
    ...  your code here
    ...
    ll.AutonomousEnd();         <<< ADD THIS LINE
  }

  public void teleopPeriodic() {
    ll.TeleopStart();           <<< ADD THIS LINE
    ...
    ...  your code here
    ...
    ll.TeleopEnd();             <<< ADD THIS LINE
  }

////////////////////////////////////////////////////////////////////////// */




package frc.robot;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;

import edu.wpi.first.wpilibj.Timer;


public class LoopLogger {

  private final static int N_FRAME_IGNORE_AVERAGING= 50;
  private final static int N_DURATION_SLOTS= 32;
  private final static int N_INITIAL_TIMES_ARRAY= 50;

  private class ThreadLogger {
    String name;
    double totalTime;    
    double startTime;
    double loopDuration;
    int lastSlot;
    int n= 0;
    final int[] durationSlots= new int[N_DURATION_SLOTS];
    final int[] initialLoopTimes= new int[N_INITIAL_TIMES_ARRAY];

    ThreadLogger( String arg_name ) {
      name= arg_name;
    }

    void Start() {
      startTime= Timer.getFPGATimestamp();
    }

    void End() {
      loopDuration= Timer.getFPGATimestamp() - startTime;
      ProcessLoopDuration();
    }

    void Use( double duration ) { 
      loopDuration= duration;
      ProcessLoopDuration();
    }

    static String LessThanDuration_String( int slot ) {
      return slot==N_DURATION_SLOTS-1 ? 
                "   inf" : 
                String.format("%6.2f", LessThanDuration(slot) );
    }

    static double LessThanDuration( int slot ) {
      slot++;
      if( slot<=3 ) {
        return slot / 4.0;
      }
      return Math.pow( 2, (slot-4)/4 ) * ( 1 + ( slot & 3 ) / 4.0 );
    }

    private void ProcessLoopDuration() {
      
      int quantumTime= (int)( loopDuration * ( 1.0 / 0.000250 ) + 0.5 );
      int slot= 0;

      if( quantumTime > 0 ) {			
        int magnitude= 32 -Integer.numberOfLeadingZeros(quantumTime) -3;		
        if( magnitude < 0 ) {		
          slot= quantumTime;	
        }		
        else {		
          slot= quantumTime >> magnitude;
          slot += 4*magnitude;
          if( slot >= N_DURATION_SLOTS ) {	
            slot= N_DURATION_SLOTS-1;
          }	
        }
      }		

      if( n < N_INITIAL_TIMES_ARRAY ) {
        initialLoopTimes[n]= slot;
      }
      durationSlots[slot]++;
      n++;
      if( n >= N_FRAME_IGNORE_AVERAGING ) {
        totalTime += loopDuration;
      }
      lastSlot= slot;
    }


    void PrintStats() {
      System.out.println ( "THREAD: " + name );
      System.out.println( "time: " + (n-N_FRAME_IGNORE_AVERAGING) + " " + totalTime + " = " + (totalTime/(n-N_FRAME_IGNORE_AVERAGING)) );
      for( int i=0; i<N_INITIAL_TIMES_ARRAY; i++ ) {
        System.out.print( initialLoopTimes[i] );
        System.out.print( " " );
      }
      System.out.println();
      for( int i=0; i<N_DURATION_SLOTS; i++ ) {
        System.out.print( durationSlots[i] );
        System.out.print( " " );
      }
      System.out.println();
    }
  }


  //private double startTime;
  private boolean bPriorEnabled = false;
  private double priorRobotStartTime= 0;

  MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
  
  private final ThreadLogger robotLogger= new ThreadLogger( "Robot" );
  private final ThreadLogger robotStartLogger= new ThreadLogger( "RobotStart" );
  private final ThreadLogger teleopLogger= new ThreadLogger( "Teleop" );
  private final ThreadLogger aggTeleopLogger= new ThreadLogger( "Agg Teleop" );
  private final ThreadLogger autonomousLogger= new ThreadLogger( "Autonomous" );
  private final ThreadLogger aggAutonomousLogger= new ThreadLogger( "Agg Autonomous" );

  private long nMemGain= 0;
  private long accMemGain= 0;
  private long nMemDrop= 0;
  private long priorUsedMemory= 0;

  LoopLogger() {};

  void SelfTest() {
    System.out.println( "LL SelfTest");

    ThreadLogger tl= new ThreadLogger( "test" );

    for( double f=0.000250; f<0.008; f+=0.000250 ) {
      tl.loopDuration= f;
      tl.ProcessLoopDuration();
      System.out.println( "= " + f + " " + tl.lastSlot );
    }

    tl.PrintStats();
  }

  void RobotStart() {
    
    robotLogger.Start();
    
    if( bPriorEnabled ) {
      double periodDelay= robotLogger.startTime - priorRobotStartTime - 0.020;
      
      if( periodDelay < 0.0 ) {
        periodDelay= 0.0;
      }

      robotStartLogger.Use( periodDelay );
    }

    priorRobotStartTime= robotLogger.startTime;
  }

  void RobotEnd( boolean bEnabled ) {

    long usedMemory = memoryBean.getHeapMemoryUsage().getUsed();    

    if( bEnabled ) {

      bPriorEnabled= true;

      robotLogger.End();

      long deltaMemory= usedMemory - priorUsedMemory;

      if( deltaMemory < 0 ) {
        nMemDrop++;
      }
      else {
        nMemGain++;
        accMemGain += deltaMemory;
      }

    }
    else if( bPriorEnabled ) {
      bPriorEnabled= false;

      PrintStats();

      System.out.println( "current used memory " + usedMemory );
      System.out.println( "nDrop " + nMemDrop );
      System.out.println( "nGain " + nMemGain );
      System.out.println( "gained per iter " + ( accMemGain * 1.0 / nMemGain ) );      
    }

    priorUsedMemory= usedMemory;
  }

  void TeleopStart() {
    teleopLogger.Start();
  }

  void TeleopEnd() {
    teleopLogger.End();
    aggTeleopLogger.Use( robotLogger.loopDuration + teleopLogger.loopDuration );
  }

  void AutonomousStart() {
    autonomousLogger.Start();
  }

  void AutonomousEnd() {
    autonomousLogger.End();
    aggAutonomousLogger.Use( robotLogger.loopDuration + autonomousLogger.loopDuration );
  }

  String StatsAsString() {
    String st= "===== LOOP LOGGER STATS =====\n";
    
    st += "=== Initial Durations\n";
    for( int i=0; i<N_INITIAL_TIMES_ARRAY; i++ ) {
      st += String.format( "%2d %2d %2d %2d %2d %2d\n", 
                            i+1,
                            robotLogger.initialLoopTimes[i],
                            autonomousLogger.initialLoopTimes[i],
                            aggAutonomousLogger.initialLoopTimes[i],
                            teleopLogger.initialLoopTimes[i],
                            aggTeleopLogger.initialLoopTimes[i] );
    }
    st += "== Timing Slots\n Below  StOff  Robot  AutoOp    Agg  TeleOp    Agg\n";
    for( int i=0; i<N_DURATION_SLOTS; i++ ) {
      st += String.format( "%s %6d %6d  %6d %6d  %6d %6d\n", 
                            ThreadLogger.LessThanDuration_String( i ),
                            robotStartLogger.durationSlots[i],
                            robotLogger.durationSlots[i],
                            autonomousLogger.durationSlots[i],
                            aggAutonomousLogger.durationSlots[i],
                            teleopLogger.durationSlots[i],
                            aggTeleopLogger.durationSlots[i] );
    }
    if( aggAutonomousLogger.n > N_FRAME_IGNORE_AVERAGING ) {
      st += String.format( "  auto time: %f across %d = %f ms per\n",
                              aggAutonomousLogger.totalTime,
                              aggAutonomousLogger.n-N_FRAME_IGNORE_AVERAGING,
                              1000.0 * aggAutonomousLogger.totalTime / ( aggAutonomousLogger.n - N_FRAME_IGNORE_AVERAGING ) );
    } else {
      st += "  auto time: ---\n";
    }
    if( aggTeleopLogger.n > N_FRAME_IGNORE_AVERAGING ) {
      st += String.format( "teleop time: %f across %d = %f ms per\n",
                            aggTeleopLogger.totalTime,
                            aggTeleopLogger.n-N_FRAME_IGNORE_AVERAGING,
                            1000.0 * aggTeleopLogger.totalTime / ( aggTeleopLogger.n - N_FRAME_IGNORE_AVERAGING ) );
    } else {
      st += "teleop time: ---\n";
    }

    return st;
  }

  void PrintStats() {
    System.out.print( StatsAsString() );
  }
}