package calibrator;

import java.io.OutputStream;
import java.lang.invoke.MethodHandles;
import java.util.logging.Filter;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.StreamHandler;

/**
 * Helper class for setting up the application loggers.
 * 
 * Handle errors to "err"
 * 
 * Handle all to "out"
 */

public final class Loggers {
    static { // configured logger hasn't started yet so use system out and err; you can log here but
            // messages disappear probably because of the reset
        System.out.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName());
        System.err.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // default values set here and setupLoggers can override or not
    private static OutputStream errLog = System.err;

    private static String outFormat = "%4$-7s [%3$s %2$s] %5$s %6$s%n";
    private static String outHeader = "";
    private static String outTail = "";
    private static Level outLevel = Level.ALL;
    
    private static String errFormat = "%1$tY-%1$tm-%1$td %1$tH:%1$tM:%1$tS.%1$tL %4$-7s [%3$s %2$s] %5$s %6$s%n";
    private static String errHeader = "NEW BEGINNINGS\n";
    private static String errTail = "THE END\n";
    private static Level errLevel = Level.WARNING;

  private Loggers() {
    throw new UnsupportedOperationException("This is a utility class");
  }

  public static void setupLoggers(OutputStream errLog, String outFormat, String outHeader, String outTail, Level outLevel,
    String errFormat, String errHeader, String errTail, Level errLevel) {
        Loggers.errLog = errLog;
        Loggers.outFormat = outFormat;
        Loggers.outHeader = outHeader;
        Loggers.outTail = outTail;
        Loggers.outLevel = outLevel;
        Loggers.errFormat = errFormat;
        Loggers.errHeader = errHeader;
        Loggers.errTail = errTail;
        Loggers.errLevel = errLevel;

        setupLoggers();
    }

   public static void setupLoggers() {
    //Set up the global level logger. This handles IO for all loggers.
    LogManager.getLogManager().reset(); // remove all handlers; same as a removeHandler loop
    final Logger globalLogger = LogManager.getLogManager().getLogger(""); // handler called null name
    
    // // Remove the default handlers that stream to System.err
    // for (Handler handler : globalLogger.getHandlers()) {
    //   globalLogger.removeHandler(handler);
    // }

    LogFormatter outFormatter = new LogFormatter(outFormat, outHeader, outTail);
    final StreamHandler stdout = new StreamHandler(System.out, outFormatter)
     {
    //final StreamHandler stdout = new StreamHandler(System.out, new SimpleFormatter()) {
        @Override
        public synchronized void publish(final LogRecord record) {
          super.publish(record);
          // For some reason this doesn't happen automatically.
          // This will ensure we get all of the logs printed to the console immediately instead of at shutdown
          flush(); // flush isn't in the default publish
        }
      }
      ;

    // used for all SimpleFormatters [new SimpleFormatter()] so if you want different formats different formatters must be used.
    // System.setProperty("java.util.logging.SimpleFormatter.format",
    // "%1$tY-%1$tm-%1$td %1$tH:%1$tM:%1$tS.%1$tL %4$-7s [%3$s %2$s] %5$s %6$s%n");

    LogFormatter errFormatter = new LogFormatter(errFormat, errHeader, errTail);
    final StreamHandler stderr = new StreamHandler(errLog, errFormatter)
     {
        @Override
        public synchronized void publish(final LogRecord record) {
          super.publish(record);
          // For some reason this doesn't happen automatically.
          // This will ensure we get all of the logs printed to the console immediately instead of at shutdown
          flush(); // flush isn't in the default publish
        }
      }
      ;
    
    // 1st stage filters
    // OFF, SEVERE, WARNING, INFO, CONFIG, FINE, FINER, FINEST, ALL
    stdout.setLevel(outLevel);
    stderr.setLevel(errLevel);
    
    // 2nd stage filter for out
    Filter outFilter = new Filter() {
        public boolean isLoggable(LogRecord record) {
            String ClassName = record.getSourceClassName();
            // System.out.println(">" + ClassName + "<");
            // don't want to see any java awt or swing messages - might be slightly dangerous, though. could suppress bad ones
            if(ClassName.startsWith("java.awt")) return false;
            if(ClassName.startsWith("java.io")) return false;
            if(ClassName.startsWith("sun")) return false;
            if(ClassName.startsWith("javax")) return false;            
            if(Main.outOverride2ndStageClassFilter) return true; // this filter allows all messages it sees
            // var method = record.getSourceClassName().replaceFirst("team4237.", "");

            if(Main.classLevels.get(ClassName) != null) // use level if specified by user
            {
                if(record.getLevel().intValue() >= Main.classLevels.get(ClassName).intValue())
                    return true;
                    else return false;
            }
            return true;
        }
    };
    
    // 2nd stage filter for err
    Filter errFilter = new Filter() {
        public boolean isLoggable(LogRecord record) {
            String ClassName = record.getSourceClassName();
            // System.out.println(">" + ClassName + "<");
            // don't want to see any java awt or swing messages - might be slightly dangerous, though. could suppress bad ones
            if(ClassName.startsWith("java.awt")) return false;
            if(ClassName.startsWith("java.io")) return false;
            if(ClassName.startsWith("sun")) return false;
            if(ClassName.startsWith("javax")) return false;            
            if(Main.errOverride2ndStageClassFilter) return true; // this filter allows all messages it sees
            if(Main.classLevels.get(ClassName) != null) // use level if specified by user
            {
                if(record.getLevel().intValue() >= Main.classLevels.get(ClassName).intValue())
                    return true;
                    else return false;
            }
            // if(record.getMessage().equals("robot driving messages not being sent to roboRIO")) return false;
            return true;
        }
    };

    // record.getSourceClassName().startsWith("org.netbeans.modules.parsing.impl.indexing.LogContext")
    // handler.setFilter(record -> record.getLevel().equals(Level.SEVERE));
    // record.getLevel().intValue() >= Level.FINE.intValue()
    // Throwable t = record.getThrown();

    // if (t != null) {
    //     return true;
    // }

    // && !logRecord.getLevel().equals(CONFIG)
    // && currentFilter.map(f -> f.isLoggable(logRecord)).orElse(true); // Honour existing filter if exists
    // && !filter.isLoggable(record)
    stdout.setFilter(outFilter);
    stderr.setFilter(errFilter);

    globalLogger.addHandler(stdout);
    globalLogger.addHandler(stderr);

    // stdout.setFormatter(new SimpleFormatter()); //log in text, not xml
    // stderr.setFormatter(new SimpleFormatter()); //log in text, not xml

    // first level master filter default isn't ALL
    // set to ALL so ALL messages go to the handlers which have their own level limits
    globalLogger.setLevel(Level.ALL);
    
    System.out.flush();
    System.err.flush(); // try to force the header out before anything else happens

    globalLogger.config("Logger configuration done."); // Log that we are done setting up the logger
   }
}
/*

java.util.logging Levels	ALL FINEST FINER	FINE	INFO	CONFIG  WARNING	SEVERE	OFF

Modifier and Type	Field and Description
static Level	Level.ALL
ALL indicates that all messages should be logged.
static Level	Level.CONFIG
CONFIG is a message level for static configuration messages.
static Level	Level.FINE
FINE is a message level providing tracing information.
static Level	Level.FINER
FINER indicates a fairly detailed tracing message.
static Level	Level.FINEST
FINEST indicates a highly detailed tracing message.
static Level	Level.INFO
INFO is a message level for informational messages.
static Level	Level.OFF
OFF is a special level that can be used to turn off logging.
static Level	Level.SEVERE
SEVERE is a message level indicating a serious failure.
static Level	Level.WARNINGING
WARNING is a message level indicating a potential problem.
*/

/*
 Logger.getLogger(LoggingSpanExporter.class.getName())
      .addHandler(
          new StreamHandler(System.err, new SimpleFormatter()) {
            @Override
            public synchronized void flush() {
              flushed.set(true);
            }
          });
*/

// import java.io.File;
// import java.io.IOException;
// import java.time.LocalDateTime;
// import java.time.format.DateTimeFormatter;
// import java.util.Locale;
// import java.util.logging.FileHandler;
// import java.util.logging.Handler;
// import java.util.logging.Level;
// import java.util.logging.LogManager;
// import java.util.logging.LogRecord;
// import java.util.logging.Logger;
// import java.util.logging.SimpleFormatter;
// import java.util.logging.StreamHandler;

// final class Loggers {

//     private Loggers() {
//       throw new UnsupportedOperationException("This is a utility class");
//     }
  
    
//     // Sets up loggers to print to stdout (rather than stderr) and log to ~/PathWeaver/pathweaver.time.log
   
  
//     public static void setupLoggers() throws IOException {
//       //Set up the global level logger. This handles IO for all loggers.
//       final Logger globalLogger = LogManager.getLogManager().getLogger("");
  
//       // Remove the default handlers that stream to System.err
//       for (Handler handler : globalLogger.getHandlers()) {
//           System.out.println("globalHandler being removed " + handler.toString());
//         globalLogger.removeHandler(handler);
//       }
  
//   System.out.println("resource bundle name " + globalLogger.getResourceBundleName());
//       String directory = System.getProperty("user.home") + "/PathWeaver/logs";
//         File folder = new File(directory);
//       if (!folder.exists()) {
//         folder.mkdir();
//       }
//       String time = DateTimeFormatter.ofPattern("YYYY-MM-dd-HH.mm.ss", Locale.getDefault()).format(LocalDateTime.now());
//       final Handler fileHandler = new FileHandler(directory + "/pathweaver." + time + ".log");
  
//       fileHandler.setLevel(Level.INFO);    // Only log INFO and above to disk
//       globalLogger.setLevel(Level.CONFIG); // Log CONFIG and higher
  
//       // We need to stream to System.out instead of System.err
//       final StreamHandler sh = new StreamHandler(System.out, new SimpleFormatter()) {
//         @Override
//         public synchronized void publish(final LogRecord record) {
//           super.publish(record);
//           // For some reason this doesn't happen automatically.
//           // This will ensure we get all of the logs printed to the console immediately instead of at shutdown
//           flush();
//         }
//       };
//       sh.setLevel(Level.CONFIG); // Log CONFIG and higher to stdout
  
//       globalLogger.addHandler(sh);
//       globalLogger.addHandler(fileHandler);
//       fileHandler.setFormatter(new SimpleFormatter()); //log in text, not xml
  
//       globalLogger.config("Configuration done."); //Log that we are done setting up the logger
  
//     }
  
//   }

/*

From http://docs.oracle.com/javase/7/docs/api/java/util/logging/SimpleFormatter.html

where the arguments are:
   1 format - the java.util.Formatter format string specified in the java.util.logging.SimpleFormatter.format property or the default format.
   2 date - a Date object representing event time of the log record.
   3 source - a string representing the caller, if available; otherwise, the logger's name.
   4 logger - the logger's name.
   5 level - the log level.
   6 message - the formatted log message returned from the Formatter.formatMessage(LogRecord) method. It uses java.text formatting and does not use the java.util.Formatter format argument.
   7 thrown - a string representing the throwable associated with the log record and its backtrace beginning with a newline character, if any; otherwise, an empty string.
Since the first argument is the format string, the rest of the arguments start at index 1 (with the date first):

in the format string:
   1 date - a Date object representing event time of the log record.
   2 source - a string representing the caller, if available; otherwise, the logger's name.
   3 logger - the logger's name.
   4 level - the log level.
   5 message - the formatted log message returned from the Formatter.formatMessage(LogRecord) method. It uses java.text formatting and does not use the java.util.Formatter format argument.
   6 thrown - a string representing the throwable associated with the log record and its backtrace beginning with a newline character, if any; otherwise, an empty string.


*/
/*
        public boolean isLoggable(LogRecord record) {
            System.err.println("out >>" +
            record.getClass() + " " + // class java.util.logging.LogRecord
            record.getInstant() + " " + // 2021-05-29T08:59:30.974305Z
            record.getLevel() + " " + //  SEVERE
            record.getLoggerName() + " " + // empty string
            record.getMessage() + " " + // robot driving messages not being sent to roboRIO
            record.getMillis() + " " + // 1622278770974
            record.getParameters() + " " + // null
            record.getResourceBundle() + " " + // null
            record.getResourceBundleName() + " " + // null
            record.getSequenceNumber() + " " + // 401
            record.getSourceClassName() + " " +  // Main Loggers      TargetSelectionB
            record.getSourceMethodName() + " " + // main setupLoggers <clinit>
            record.getThreadID() + " " + // 1
            record.getThrown()+ "<<"); // null probably the thrown exception trace

*/
// System.err.println("out >>" +
// record.getLevel() + " " + //  SEVERE
// record.getLoggerName() + " " + // empty string
// record.getSourceClassName() + " " +  // Main Loggers      TargetSelectionB
// record.getSourceMethodName() + " " + // main setupLoggers <clinit>
// record.getThrown()+ "<<"); // null probably the thrown exception trace