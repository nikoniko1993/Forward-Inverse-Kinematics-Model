# Forward-Inverse-Kinematics-Model
COMS 4160 PA2 README.txt
Nicole Seleme (ns2873)

********************* Components ***********************

*4 classes:
	- 2 executables: 
		> ForwKinArm.java
		> InverseKin.java

	- 2 Object classes:
		> Camera.java
		> Joint.java

* Additional JAR files:
	- Slick.jar (For textures)
	- EJML (Efficient Java Matrix Library)
	- lwjgl library

- (Please note that the rotation of the Camera has been disabled for both executables. However, the camera can still move around the X-Z plane and up and down the Y axis)

***********************************************************
************** Part A: FORWARD KINEMATICS *****************
***********************************************************
- Input Commands:

		COMMAND 				KEYBOARD KEY

		Switch model			TAB

		Select Joint			1 - 4 (Robot Arm)

		Select Finger			1 - 5 (Hand)

		Select Knuckle			J, K, L

		Rotate Clockwise		N

		Rotate CountClockWise	M

		Move Camera 			W, A, S, D, SHIFT, SPACEBAR

		Quit					ESC


- Please compile the ForwKinArm.java, Joint.java, and Camera.java classes with the slick and lwjgl libraries. 

- For this part, I programmed a Robot arm with 4 joints and A hand with 5 fingers, 3 knuckles each. The selected 
Joint is highlighted with purple, and that is the one that, when rotated, will affect the rotation of all its children.

- Both models are rendered at the same time onscreen, and the user can switch between which one to alter with the TAB key.
	
	- If the selected model is the arm, then the user can switch between joints by pressing the 1-4 keys. 
	- If the selected model is the hand , then pressing the 1 - 5 keys will switch between the starting knuckles of each finger.
		- Once a finger has been selected, the user can traverse the knuckles of that finger with the J, K, and L keys. 

- Pressing the N and M keys will rotate the currently selected joint clockwise and counter-clockwise respectively.

- For this program it was assumed that each node could have at most one child.

- The Camera class was provided by the started code, and was only modified to change the starting position of the program for a better look at the models.

- The Slick.util library was used to apply the starry texture to the skybox. Also, I used a display list to make the skybox draw faster. 

- I applied some lighting, similar to that of the first assignment.

***********************************************************
************** Part B: INVERSE KINEMATICS *****************
***********************************************************

- Input Commands:

		COMMAND 				KEYBOARD KEY

		Switch model			TAB

		Select Joint			1 - 4 (Robot Arm (1))

		Select Joint			0 - 9 (Long Robot Arm (2))

		Select Target Pos 		Mouse click

		Move Camera 			W, A, S, D, SHIFT, SPACEBAR

		Quit					ESC


- Please compile the Inverse.java, Joint.java, and Camera.java classes with the slick, EJML, and lwjgl libraries. 

- This program only renders one model at a time, depending on the currently selected model.

- The user's selected joint will be highlighted in purple. When selecting a new target position, the end tip of the cylinder that BEGINS from the selected joint will be moved to that position. 

- Cases where the target position are out of reach of the arm are handled adequately, with the apropriate links stretching out as far as possible in that direction.

- The dampening value used was = 0.5f

- The allowed error was = 0.1f

- Here the two models are two separate robot arms, one with 4 joints, as in the ForwKin class, and another, with 10 joints. 

- When the user switches models with the TAB key, the selected model will be rendered.

- When the long arm is selected, please note that pressing the 0 key will select the last joint, furthest away from the origin. 

- Again, the same skybox and lighting settings are used for this class. 

- The slick class was used to texturize the skybox, the Ejml library was used to calculate the Jacobian matrices and manipulate all objects that would have been Vectors

- The starting position of the models has been hardcoded at the beginning of the program.





