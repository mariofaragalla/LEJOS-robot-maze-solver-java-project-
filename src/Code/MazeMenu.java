package Code;

import lejos.hardware.lcd.*;
import lejos.robotics.Color;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class MazeMenu { // this class is implemented to display the menu on the robot screen
	// Define Constants
	private final int LCD_LENGTH = LCD.DISPLAY_CHAR_DEPTH; // get the length of the lcd screen
	// set the origin (x,y) of the lcd to be 0,0
	private final int X_ORIGIN = 0;
	private final int Y_ORIGIN = 0;

	private final int FIRST_ROW = 0; // Number corresponding to the first row
	private final int LAST_ROW = 7; // Number corresponding to the last row

	private final int HALF_SECOND = 500; // Half a second in milli seconds

	// Initialize the strings that will be displayed on the screen
	private String welcome = "Please select color: ";

	private String red = "Red";
	private String green = "Green";
	private String blue = "Blue";
	private String yellow = "Yellow";
	private String magenta = "Magenta";
	private String orange = "Orange";
	private String cyan = "Cyan";

	private String sign = ">"; // Identify the cursor shape

	// Initialize variables:

	// define 2 variables for current x and y positions
	private int x = X_ORIGIN;
	private int y = Y_ORIGIN;

	private int currentRow = 0; // A counter that traces the current row of the cursor

	// define the integer & the string that correspond to the color which will be
	// selected from the Menu
	private int selectedColor = Color.NONE;
	private String selectedColour = "none";

	public void startMenu() { // Main method to display Menu
		display(); // display menu
		while (true) { // Always do the following
			Button.waitForAnyPress(); // Wait for any button to be pressed.
			checkButton(); // check which button is pressed and make the corresponding action
			if (isEnter()) { // if the OK button is pressed
				break; // that means that a color is chosen so we must get out of the loop
			}
			display(); // keep displaying everything until the OK button is pressed
		}
	}

	private void display() { // Method to display the Menu

		// print all strings on the screen on corresponding lines
		LCD.drawString(welcome, X_ORIGIN, Y_ORIGIN);

		LCD.drawString(red, sign.length(), ++y);
		LCD.drawString(green, sign.length(), ++y);
		LCD.drawString(blue, sign.length(), ++y);
		LCD.drawString(yellow, sign.length(), ++y);
		LCD.drawString(magenta, sign.length(), ++y);
		LCD.drawString(orange, sign.length(), ++y);
		LCD.drawString(cyan, sign.length(), ++y);

		y = Y_ORIGIN; // reset y to the origin position
	}

	private void checkButton() { // check which button is pressed
		// check if Up button is pressed and counter in range
		if (Button.UP.isDown() && currentRow > FIRST_ROW && currentRow <= LAST_ROW) {
			LCD.drawString(" ", x, currentRow); // Delete cursor of the current row
			LCD.drawString(sign, x, --currentRow); // Print cursor on the previous row
		} else if (Button.DOWN.isDown() && currentRow >= FIRST_ROW && currentRow < LAST_ROW) {
			// if down button is pressed and counter in range
			LCD.drawString(" ", x, currentRow); // Delete cursor of the current row
			LCD.drawString(sign, x, ++currentRow); // print cursor one the next row
		}
	}

	private boolean isEnter() { // checks if OK button is pressed and store the selected color
		// if OK button is pressed and counter in range
		if (Button.ENTER.isDown() && currentRow > FIRST_ROW && currentRow <= LAST_ROW) {
			// see which colour corresponds to the current selected row/line and store the
			// Colours value
			switch (currentRow) {
			case 1:
				selectedColor = Color.RED;
				selectedColour = "Red";
				break;
			case 2:
				selectedColor = Color.GREEN;
				selectedColour = "Green";
				break;
			case 3:
				selectedColor = Color.BLUE;
				selectedColour = "Blue";
				break;
			case 4:
				selectedColor = Color.YELLOW;
				selectedColour = "Yellow";
				break;
			case 5:
				selectedColor = Color.MAGENTA;
				selectedColour = "Magenta";
				break;
			case 6:
				selectedColor = Color.ORANGE;
				selectedColour = "Orange ";
				break;
			case 7:
				selectedColor = Color.CYAN;
				selectedColour = "Cyan";
				break;
			default:
				break;
			}
			LCD.clear(); // Clear the menu to display new values
			y = (LCD_LENGTH / 2) - 2; // assign y to print 2 lines before the middle of the screen
			LCD.drawString("Color : " + selectedColour + " " + selectedColor, x, y); // display selected color
			y = y + 2; // assign y to print in the middle of the screen.
			LCD.drawString("Start Solving Maze...", x, y);

			return true;
		}
		return false;
	}

	public int getMenuColor() { // get the selected color by the user
		return selectedColor;
	}

	public void displayEndMessage() { // display this message when the robot find the selected color in the maze
		LCD.clear();
		y = (LCD_LENGTH / 2) - 2; // assign y to print 2 lines before the middle of the screen
		Delay.msDelay(HALF_SECOND); // Wait half a second
		LCD.drawString("Congratulations !", x, y);
		Delay.msDelay(HALF_SECOND);// Wait half a second
		y = LCD_LENGTH / 2; // assign y to print in the middle of the screen.
		LCD.drawString(selectedColour + " is Found", x, y);
	}

}
