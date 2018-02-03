package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import gnu.io.*;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.TooManyListenersException;

/**
 * Class to output Serial data for Arduino
 * Hopefully can be moved into a utility
 */
public class HeadsUpDisplay extends Subsystem implements SerialPortEventListener {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	// Based on these instructions: https://blog.henrypoon.com/blog/2011/01/01/serial-communication-in-java-with-example-program/
	
	//for containing the ports that will be found
    @SuppressWarnings("rawtypes")
	private Enumeration ports = null;
    //map the port names to CommPortIdentifiers
    private HashMap<String, CommPortIdentifier> portMap = new HashMap<>();

    //this is the object that contains the opened port
    private CommPortIdentifier selectedPortIdentifier = null;
    private SerialPort serialPort = null;

    //input and output streams for sending and receiving data
    private InputStream input = null;
    private OutputStream output = null;

    //the timeout value for connecting with the port
    final static int TIMEOUT = 2000;

    //some ascii values for for certain things
    final static int SPACE_ASCII = 32; // Shouls be able to remove some of these
    final static int DASH_ASCII = 45;
    final static int NEW_LINE_ASCII = 10;

    //a string for recording what goes on in the program
    //this string is written to the GUI
    String logText = "";
    
    //search for all the serial ports
    //pre style="font-size: 11px;": none
    //post: adds all the found ports to a combo box on the GUI
    public void searchForPorts()
    {
        ports = CommPortIdentifier.getPortIdentifiers();
        
        while (ports.hasMoreElements()) {
            CommPortIdentifier curPort = (CommPortIdentifier)ports.nextElement();
            //get only serial ports
            if (curPort.getPortType() == CommPortIdentifier.PORT_SERIAL) portMap.put(curPort.getName(), curPort); // Want to display on the SmartDashboard
        }
    }
    
    /**
     * Connect to the selected port (no current way to select port, default to COM13)
     * Ports must be already found by searchForPorts method
     * The connected Comm port is stored in commPort, otherwise, an exception is generated
     */
    public void connect()
    {
        String selectedPort = "COM13"; // Somehow change to be settable on the SmartDashboard
        selectedPortIdentifier = (CommPortIdentifier)portMap.get(selectedPort);

        CommPort commPort = null;

        try
        {
            //the method below returns an object of type CommPort
            commPort = selectedPortIdentifier.open("TigerControlPanel", TIMEOUT);
            //the CommPort object can be casted to a SerialPort object
            serialPort = (SerialPort)commPort;

            //CODE ON SETTING BAUD RATE ETC OMITTED
        }
        catch (PortInUseException e)
        {
            logText = selectedPort + " is in use. (" + e.toString() + ")"; // Output to SmartDashboard or Driver Station
        }
        catch (Exception e)
        {
            logText = "Failed to open " + selectedPort + "(" + e.toString() + ")"; // Output to SmartDashboard or Driver Station
        }
    }
    
    /**
     * Open the input and output streams
     * @return true if successfull, false if not
     */
    public boolean initIOStream()
    {
        //return value for whether opening the streams is successful or not
        boolean successful = false;

        try {
            //
            input = serialPort.getInputStream(); // Should be able to remove input stream
            output = serialPort.getOutputStream();
            writeData(0, 0);

            successful = true;
            return successful;
        }
        catch (IOException e) {
            logText = "I/O Streams failed to open. (" + e.toString() + ")"; // Output to SmartDashboard or Driver Station
            return successful;
        }
    }
    
    /**
     * Disconnects the serial port and frees up resources
     */
    public void disconnect()
    {
        //close the serial port
        try
        {
            writeData(0, 0); // Should be able to remove this, don't need to write any data

            serialPort.removeEventListener(); // Don't have any event listeners in the first place, not listening for data
            serialPort.close();
            input.close();
            output.close();
            
            logText = "Disconnected"; // Output to SmartDashboard or Driver Station
        }
        catch (Exception e)
        {
            logText = "Failed to close " + serialPort.getName() + "(" + e.toString() + ")"; // Output to SmartDashboard or Driver Station
        }
    }
    
    /**
     * Send data to the connected Serial Device
     * @param deviceID ID to send device data to (to be implemented)
     * @param value value to send
     */
    public void writeData(int deviceID, int value)
    {
        try
        {
            output.write(deviceID);
            output.flush();
            //this is a delimiter for the data
            output.write(DASH_ASCII);
            output.flush();

            output.write(value);
            output.flush();
            //will be read as a byte so it is a space key
            output.write(SPACE_ASCII);
            output.flush();
        }
        catch (Exception e)
        {
            logText = "Failed to write data. (" + e.toString() + ")"; // Output to SmartDashboard or Driver Station
        }
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	@Override
	public void serialEvent(SerialPortEvent arg0) {
		// TODO Auto-generated method stub
		
	}
}

