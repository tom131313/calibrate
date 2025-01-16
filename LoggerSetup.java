package Guidance;

import java.io.IOException;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

/**
 * Handle all log message levels
 * 
 * Direct all messages to the log file without colors.
 * 
 * Direct all messages to the consle (terminal) with colors.
 */
public class LoggerSetup {

    private static Logger LOGGER;  

    public static Logger setupLogger() {

    LOGGER = Logger.getLogger("");
    LOGGER.setUseParentHandlers(false);
    // remove any default handlers
    Handler[] handlers = LOGGER.getHandlers();
    for(Handler handler : handlers) {
        LOGGER.removeHandler(handler);
    }

    ConsoleHandler consoleHandler = new ConsoleHandler();
    FileHandler  logFile = null;
    try {
        logFile = new FileHandler("log.txt",true);
    } catch (SecurityException | IOException e) {
        e.printStackTrace();
    }

    System.setProperty("java.util.logging.SimpleFormatter.format",
    "%1$tY-%1$tm-%1$td %1$tH:%1$tM:%1$tS.%1$tL %4$-7s [%3$s %2$s] %5$s %6$s%n");

    consoleHandler.setFormatter(new LoggerColoredSimpleFormatter());

    logFile.setFormatter(new SimpleFormatter());

    LOGGER.addHandler(consoleHandler);

    LOGGER.addHandler(logFile);

    handlers = LOGGER.getHandlers();
    for(Handler handler : handlers) {
        handler.setLevel(Cfg.loggerMinimumLevel);
    }
    LOGGER.setLevel(Cfg.loggerMinimumLevel);
    Logger.getLogger("").getHandlers()[0].setLevel(Level.FINEST);

    return LOGGER;
    }
    // test logging
    // LOGGER.finest("a finest message");
    // LOGGER.finer("a finer message");
    // LOGGER.fine("a fine");
    // LOGGER.config("a config message");
    // LOGGER.info("an info message");
    // LOGGER.warning("a warning message");
    // LOGGER.severe("a severe message");
}
