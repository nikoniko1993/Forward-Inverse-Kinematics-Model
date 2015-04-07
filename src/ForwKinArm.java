import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.io.File;
import java.io.IOException;
import java.awt.image.BufferedImage;

import javax.imageio.ImageIO;

import org.lwjgl.LWJGLException;
import org.lwjgl.Sys;
import org.lwjgl.BufferUtils;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;

import static  org.lwjgl.opengl.GL11.*;

import org.lwjgl.util.glu.GLU;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.glu.Sphere;
import org.lwjgl.util.glu.Cylinder;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;
import org.newdawn.slick.util.ResourceLoader;

public class ForwKinArm {

    String windowTitle = "Forward Kinematics";
    public boolean closeRequested = false;

    long lastFrameTime; // used to calculate delta
    
    float triangleAngle; // Angle of rotation for the triangles
    float quadAngle; // Angle of rotation for the quads
    
    public static Joint selectedJoint;
    public static ArrayList<Joint> robotArmJoints = new ArrayList<Joint>();

    public static ArrayList<Joint> handJoints = new ArrayList<Joint>();
    public static ArrayList<Joint> selectedModel;
    public static float [] robotAngles = {30, 50, -60, 0};
    public static float [] robotLengths = {1.5f, 1.5f, 1.5f, 0};
    public static float [] handAngles = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    public static float [] handLengths = {0.6f, 0.6f, 0.4f, 0.8f, 0.8f, 0.6f, 1f, 0.8f, 0.6f, 0.8f, 0.6f, 0.6f, 0.6f, 0.6f, 0.5f, };
    public static int rootAngle = 0;
    public static int knuckle = 0;
    
    //DISPLAYLIST
    int makeSkyBox;
    
    //Textures
  	Texture starry;
    
    public void run() {

        createWindow();
        getDelta(); // Initialize delta timer
        initGL();
        
        while (!closeRequested) {
            pollInput();
            updateLogic(getDelta());
            renderGL();

            Display.update();
        }
        
        cleanup();
    }
    
    private void initGL() {

        /* OpenGL */
        int width = Display.getDisplayMode().getWidth();
        int height = Display.getDisplayMode().getHeight();

        glViewport(0, 0, width, height); // Reset The Current Viewport
        glMatrixMode(GL_PROJECTION); // Select The Projection Matrix
        glLoadIdentity(); // Reset The Projection Matrix
        GLU.gluPerspective(45.0f, ((float) width / (float) height), 0.1f, 100.0f); // Calculate The Aspect Ratio Of The Window
        glMatrixMode(GL_MODELVIEW); // Select The Modelview Matrix
        glLoadIdentity(); // Reset The Modelview Matrix

        glShadeModel(GL_SMOOTH); // Enables Smooth Shading
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Black Background
        glClearDepth(1.0f); // Depth Buffer Setup
        glEnable(GL_DEPTH_TEST); // Enables Depth Testing
        glDepthFunc(GL_LEQUAL); // The Type Of Depth Test To Do
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations
        Camera.create();        
        makeArmJoints();
        makeHandJoints();
        selectedJoint = robotArmJoints.get(0); //by default, initial selected joint is root joint
        selectedModel = robotArmJoints; //by default we begin with the robot arm selected
        
        try
		{
			starry = TextureLoader.getTexture("JPG", ResourceLoader.getResourceAsStream("/Textures/OuterSpace/outerSpace.jpg"));
		}
		catch(IOException e)
		{
			System.out.println("MEH");
		}
        
      //DisplayList for Skybox
    	makeSkyBox = glGenLists(1);
    	glNewList(makeSkyBox, GL_COMPILE);
    	{
    		glBegin(GL_QUADS);
    		// Render the front quad
    		glTexCoord2f(0, 0); glVertex3f(-1.0f, -1.0f, 1.0f); //BOTTOM LEFT
    		glTexCoord2f(1, 0); glVertex3f(1.0f, -1.0f, 1.0f);	//BOTTOM RIGHT
    		glTexCoord2f(1, 1); glVertex3f(1.0f, 1.0f, 1.0f);	//TOP RIGHT
    		glTexCoord2f(0, 1); glVertex3f(-1.0f, 1.0f, 1.0f);	//TOP LEFT
    		glEnd();

    		//Render back of the quad BL, BR, TR, TL
    		glBegin(GL_QUADS);
    		glTexCoord2f(0, 0); glVertex3f(1.0f, -1.0f, -1.0f);
    		glTexCoord2f(1, 0); glVertex3f(-1.0f, -1.0f, -1.0f);
    		glTexCoord2f(1, 1); glVertex3f(-1.0f, 1.0f, -1.0f);
    		glTexCoord2f(0, 1); glVertex3f(1.0f, 1.0f, -1.0f);
    		glEnd();

    		//Render the Left quad
    		glBegin(GL_QUADS);
    		glTexCoord2f(0, 0); glVertex3f(-1.0f, -1.0f, -1.0f);
    		glTexCoord2f(1, 0); glVertex3f(-1.0f, -1.0f, 1.0f);
    		glTexCoord2f(1, 1); glVertex3f(-1.0f, 1.0f, 1.0f);
    		glTexCoord2f(0, 1); glVertex3f(-1.0f, 1.0f, -1.0f);
    		glEnd();

    		// Render the right quad
    		glBegin(GL_QUADS);
    		glTexCoord2f(0, 0); glVertex3f(1.0f, -1.0f, 1.0f);
    		glTexCoord2f(1, 0); glVertex3f(1.0f, -1.0f, -1.0f);
    		glTexCoord2f(1, 1); glVertex3f(1.0f, 1.0f, -1.0f);
    		glTexCoord2f(0, 1); glVertex3f(1.0f, 1.0f, 1.0f);
    		glEnd();


    		// Render the top quad
    		glBegin(GL_QUADS);
    		glTexCoord2f(0, 1); glVertex3f(-1.0f, 1.0f, -1.0f);
    		glTexCoord2f(0, 0); glVertex3f(-1.0f, 1.0f, 1.0f);
    		glTexCoord2f(1, 0); glVertex3f(1.0f, 1.0f, 1.0f);
    		glTexCoord2f(1, 1); glVertex3f(1.0f, 1.0f, -1.0f); 
    		glEnd();

    		// Render the bottom quad  
    		glBegin(GL_QUADS);
    		glTexCoord2f(0, 0); glVertex3f(-1.0f, -1.0f, -1.0f);
    		glTexCoord2f(0, 1); glVertex3f(1.0f, -1.0f, -1.0f);
    		glTexCoord2f(1, 1); glVertex3f(1.0f, -1.0f, 1.0f);
    		glTexCoord2f(1, 0); glVertex3f(-1.0f, -1.0f, 1.0f);
    		glEnd();
    	}
    	glEndList();
    	
    }
    
    private void updateLogic(int delta) {
        triangleAngle += 0.1f * delta; // Increase The Rotation Variable For The Triangles
        quadAngle -= 0.05f * delta; // Decrease The Rotation Variable For The Quads
    }

    private void lighting()
	{

		FloatBuffer diffuseLight = BufferUtils.createFloatBuffer(4);
		diffuseLight.put(1.0f).put(1.0f).put(1.0f).put(1.0f).flip();
		glLight(GL_LIGHT0, GL_DIFFUSE, diffuseLight);


		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		FloatBuffer matDiffAmbLight = BufferUtils.createFloatBuffer(4);
		matDiffAmbLight.put(0.5f).put(0.5f).put(0.5f).put(1.0f).flip();
		glColor3f(0.5f, 0.5f, 0.5f);
		glMaterial(GL_FRONT, GL_DIFFUSE, matDiffAmbLight);

		FloatBuffer ambientLight = BufferUtils.createFloatBuffer(4);
		ambientLight.put(0.5f).put(0.5f).put(0.5f).put(1.0f).flip();
		glLight(GL_LIGHT0, GL_AMBIENT, ambientLight);


		FloatBuffer shinyLight = BufferUtils.createFloatBuffer(4);
		shinyLight.put(1.0f).put(1.0f).put(1.0f).put(1.0f).flip();
		glLight(GL_LIGHT0, GL_SPECULAR, shinyLight);

		FloatBuffer lightPosition = BufferUtils.createFloatBuffer(4);
		lightPosition.put(-4).put(3).put(4).put(0).flip();
		glLight(GL_LIGHT0,  GL_POSITION, lightPosition);

		//model ambient will affect ALL objects!! Its like a global light
		FloatBuffer modelAmbient = BufferUtils.createFloatBuffer(4);
		shinyLight.put(1.0f).put(1.0f).put(1.0f).put(1.0f).flip();
		glLightModel(GL_LIGHT_MODEL_AMBIENT, modelAmbient);


		glLight(GL_LIGHT0, GL_AMBIENT, ambientLight);
		glLight(GL_LIGHT0, GL_POSITION, lightPosition);

		glEnable(GL_NORMALIZE);
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);


	}

    private void renderGL() {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear The Screen And The Depth Buffer
        glLoadIdentity(); // Reset The View
        Camera.apply();
        glPushMatrix();
        
        lighting();
        
        //Draw a texturized Skybox
        glPushMatrix();
        glPushAttrib(GL_ENABLE_BIT);
		glEnable(GL_TEXTURE_2D);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glScalef(15.0f, 15.0f, 15.f);
		// Just in case we make the vertices white (for texturing)
		glColor3f(1,1,1);
		starry.bind();
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S, GL_REPEAT );
		glCallList(makeSkyBox);
		glPopAttrib();
        glPopMatrix();
        
        //X AXIS
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.f); //Green
        glVertex3f( 50,0, 0);
        glVertex3f(-50,0,0);
        
		//Y AXIS
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(0,50, 0);
		glVertex3f(0,-50,0);

		//Z AXIS
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f(0,0, 50);
		glVertex3f(0,0,-50);
		glEnd();
		glPopMatrix();
        
		drawArmModel();
		drawHandModel(); 

    }

    /**
     * Poll Input
     */
    public void pollInput() {
        Camera.acceptInput(getDelta());
        // scroll through key events
        while (Keyboard.next()) {
            if (Keyboard.getEventKeyState() && selectedModel == robotArmJoints) 
            {
                if (Keyboard.getEventKey() == Keyboard.KEY_ESCAPE)
                    closeRequested = true;
                else if (Keyboard.getEventKey() == Keyboard.KEY_1)
                    selectedJoint = robotArmJoints.get(0);
                else if (Keyboard.getEventKey() == Keyboard.KEY_2)
                    selectedJoint = robotArmJoints.get(1);
                else if (Keyboard.getEventKey() == Keyboard.KEY_3)
                    selectedJoint = robotArmJoints.get(2);
                else if (Keyboard.getEventKey() == Keyboard.KEY_4)
                    selectedJoint = robotArmJoints.get(3);
                else if (Keyboard.getEventKey() == Keyboard.KEY_TAB)
                {
                	selectedModel = handJoints;
                	selectedJoint = handJoints.get(0);
                }
                

            }
            else if (Keyboard.getEventKeyState() && selectedModel == handJoints)
            {

            	if (Keyboard.getEventKey() == Keyboard.KEY_ESCAPE)
                    closeRequested = true;
            	else if (Keyboard.getEventKey() == Keyboard.KEY_1)
            	{
            		selectedJoint = handJoints.get(0);
            		knuckle = 0;
            	}
                    
                else if (Keyboard.getEventKey() == Keyboard.KEY_2)
                {
            		selectedJoint = handJoints.get(3);
            		knuckle = 3;
            	}
                else if (Keyboard.getEventKey() == Keyboard.KEY_3)
                {
            		selectedJoint = handJoints.get(6);
            		knuckle = 6;
            	}
                else if (Keyboard.getEventKey() == Keyboard.KEY_4)
                {
            		selectedJoint = handJoints.get(9);
            		knuckle = 9;
            	}
                else if (Keyboard.getEventKey() == Keyboard.KEY_5)
                {
            		selectedJoint = handJoints.get(12);
            		knuckle = 12;
            	}
                else if (Keyboard.getEventKey() == Keyboard.KEY_J)
                	selectedJoint = handJoints.get(knuckle);
                else if (Keyboard.getEventKey() == Keyboard.KEY_K)
                	selectedJoint = handJoints.get(knuckle + 1);
                else if (Keyboard.getEventKey() == Keyboard.KEY_L)
                	selectedJoint = handJoints.get(knuckle + 2);
                else if (Keyboard.getEventKey() == Keyboard.KEY_TAB)	//switches model
                {
                	selectedModel = robotArmJoints;
                	selectedJoint = robotArmJoints.get(0);
                }
            }
            
        }
        
        if (Keyboard.isKeyDown(Keyboard.KEY_N))
            selectedJoint.rotateClockwise();
        else if (Keyboard.isKeyDown(Keyboard.KEY_M))
            selectedJoint.rotateCounterClockwise();

        if (Display.isCloseRequested()) {
            closeRequested = true;
        }
    }

    public void snapshot() {
        System.out.println("Taking a snapshot ... snapshot.png");

        glReadBuffer(GL_FRONT);

        int width = Display.getDisplayMode().getWidth();
        int height= Display.getDisplayMode().getHeight();
        int bpp = 4; // Assuming a 32-bit display with a byte each for red, green, blue, and alpha.
        ByteBuffer buffer = BufferUtils.createByteBuffer(width * height * bpp);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer );

        File file = new File("snapshot.png"); // The file to save to.
        String format = "PNG"; // Example: "PNG" or "JPG"
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
   
        for(int x = 0; x < width; x++) {
            for(int y = 0; y < height; y++) {
                int i = (x + (width * y)) * bpp;
                int r = buffer.get(i) & 0xFF;
                int g = buffer.get(i + 1) & 0xFF;
                int b = buffer.get(i + 2) & 0xFF;
                image.setRGB(x, height - (y + 1), (0xFF << 24) | (r << 16) | (g << 8) | b);
            }
        }
           
        try {
            ImageIO.write(image, format, file);
        } catch (IOException e) { e.printStackTrace(); }
    }
    
    /** 
     * Calculate how many milliseconds have passed 
     * since last frame.
     * 
     * @return milliseconds passed since last frame 
     */
    public int getDelta() {
        long time = (Sys.getTime() * 1000) / Sys.getTimerResolution();
        int delta = (int) (time - lastFrameTime);
        lastFrameTime = time;
     
        return delta;
    }

    private void createWindow() {
        try {
            Display.setDisplayMode(new DisplayMode(640, 480));
            Display.setVSyncEnabled(true);
            Display.setTitle(windowTitle);
            Display.create();
        } catch (LWJGLException e) {
            Sys.alert("Error", "Initialization failed!\n\n" + e.getMessage());
            System.exit(0);
        }
    }
    
    /**
     * Destroy and clean up resources
     */
    private void cleanup() {
        Display.destroy();
    }
    
    public static void main(String[] args) {
        new ForwKinArm().run();
    }
    
    public static void drawCircle(float radius)
    {
    	glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0f, 0f, 0f);
        
        //counts how many points in the circle i want to draw
        float deltaAng = (float)(2*Math.PI/50f);
        for(int i = 0; i <= 50; i++)
        {
            glVertex3f((float)(radius * Math.cos(deltaAng*i)),
                    (float)(radius * Math.sin(deltaAng*i)), 0f);
        }
        glEnd();
    }
    
    public static void makeArmJoints()
    {
    	Joint prevJoint;
    	Joint currentJoint;
    	for(int i = 0; i < 4; i++)
    	{
    		currentJoint = new Joint(robotLengths[i], robotAngles[i]);
    		//add the joint to the Master Joint list
    		robotArmJoints.add(currentJoint);
    		if((i - 1) >= 0) //if it is NOT the FIRST joint 
    		{
    			//proceed to add it as a child of the preceding Joint
    			prevJoint = robotArmJoints.get(i -1);//get parent
    			prevJoint.childrenJoint = currentJoint; //add the newly created node as its child
    		}
    	}
    }
    
    public static void makeHandJoints()
    {
    	Joint prevJoint;
    	Joint currentJoint;
    	for(int i = 0; i < 15; i++)
    	{
    		currentJoint = new Joint(handLengths[i], handAngles[i]);
    		handJoints.add(currentJoint);
    		if((i%3) != 0)
    		{
    			prevJoint = handJoints.get(i-1);
    			prevJoint.childrenJoint = currentJoint;
    		}
    	}
    }
    
    public static void drawArmModel()
    {
    	//Draw BASE CIRCLE
    	glPushMatrix();
    	glTranslatef(2, 0, 0); //DRAW ENTIRE MODEL A LITTLE TO THE RIGHT
		glRotatef(90, 1f, 0f, 0f); 
		drawCircle(1.5f);
        glPopMatrix();
        
        glPushMatrix();
        glTranslatef(2, 0, 0);
        for(int i = 0; i < 4; i++)
        {
        	glColor3f(1.0f, 0.0f, 0.0f);
        	
        	if(selectedJoint == robotArmJoints.get(i))
        		glColor3f(1.0f, 0.0f, 1.0f);
        	Sphere sphere = new Sphere();
        	sphere.draw(0.3f, 30, 30);
    		
    		//ROTATE by joint's angle
    		glRotatef(robotArmJoints.get(i).angle, 0, 0, 1);
    		
    		//draw next link
    		glPushMatrix();
    		glColor3f(1.0f, 1.0f, 0.0f);
    			//align it with the x axis
    		glRotatef(90, 0, 1f, 0);
    		
    		Cylinder bone1 = new Cylinder();
    		bone1.draw(0.1f, 0.1f, robotArmJoints.get(i).length, 30, 30);
    		glPopMatrix();
    		
    		glTranslatef(robotArmJoints.get(i).length, 0, 0);
    		
    		//HERE i should get the position of the LAST joint I suppose?
        }
		glPopMatrix();
    }
    
    public static void drawHandModel()
    {
    	
    	glPushMatrix();
    	glTranslatef(-3, 0, 0);
    	
    	//APPLY TO QUAD
        glPushMatrix();
        glScalef(1f, 1f, 0.2f);
        
        glBegin(GL_QUADS); // Start Drawing The Cube
        glColor3f(0.0f, 1.0f, 1.0f); // Set The Color To Green
        glVertex3f(1.0f, 1.0f, -1.0f); // Top Right Of The Quad (Top)
        glVertex3f(-1.0f, 1.0f, -1.0f); // Top Left Of The Quad (Top)
        glVertex3f(-1.0f, 1.0f, 1.0f); // Bottom Left Of The Quad (Top)
        glVertex3f(1.0f, 1.0f, 1.0f); // Bottom Right Of The Quad (Top)

        glVertex3f(1.0f, -1.0f, 1.0f); // Top Right Of The Quad (Bottom)
        glVertex3f(-1.0f, -1.0f, 1.0f); // Top Left Of The Quad (Bottom)
        glVertex3f(-1.0f, -1.0f, -1.0f); // Bottom Left Of The Quad (Bottom)
        glVertex3f(1.0f, -1.0f, -1.0f); // Bottom Right Of The Quad (Bottom)

        glVertex3f(1.0f, 1.0f, 1.0f); // Top Right Of The Quad (Front)
        glVertex3f(-1.0f, 1.0f, 1.0f); // Top Left Of The Quad (Front)
        glVertex3f(-1.0f, -1.0f, 1.0f); // Bottom Left Of The Quad (Front)
        glVertex3f(1.0f, -1.0f, 1.0f); // Bottom Right Of The Quad (Front)

        glVertex3f(1.0f, -1.0f, -1.0f); // Bottom Left Of The Quad (Back)
        glVertex3f(-1.0f, -1.0f, -1.0f); // Bottom Right Of The Quad (Back)
        glVertex3f(-1.0f, 1.0f, -1.0f); // Top Right Of The Quad (Back)
        glVertex3f(1.0f, 1.0f, -1.0f); // Top Left Of The Quad (Back)

        glVertex3f(-1.0f, 1.0f, 1.0f); // Top Right Of The Quad (Left)
        glVertex3f(-1.0f, 1.0f, -1.0f); // Top Left Of The Quad (Left)
        glVertex3f(-1.0f, -1.0f, -1.0f); // Bottom Left Of The Quad (Left)
        glVertex3f(-1.0f, -1.0f, 1.0f); // Bottom Right Of The Quad (Left)

        glVertex3f(1.0f, 1.0f, -1.0f); // Top Right Of The Quad (Right)
        glVertex3f(1.0f, 1.0f, 1.0f); // Top Left Of The Quad (Right)
        glVertex3f(1.0f, -1.0f, 1.0f); // Bottom Left Of The Quad (Right)
        glVertex3f(1.0f, -1.0f, -1.0f); // Bottom Right Of The Quad (Right)
        glEnd(); // Done Drawing The Quad
        glPopMatrix();
        
        //glPushMatrix(); //1 we are still at palm center
    	glTranslatef(-1.3f, 1f, 0);
    	//glRotatef(90, 0, 0, 1f);
    	//here we are at the beginning of the pinky finger
    	glPushMatrix(); //2
    	for(int i = 0; i < 15; i++)
    	{
    		if(i%3 != 0)
    		{
    			//glPopMatrix();
    			glTranslatef(0, handJoints.get(i-1).length, 0);
    			//glPushMatrix();
    		}
    		else if(i != 12)
    		{
    			glPopMatrix();
    			glTranslatef(0.5f, 0f, 0f); //moves us to next finger
    			glPushMatrix();
    		}
    		else
    		{
    			glPopMatrix();
    			glTranslatef(0.5f, -0.8f, 0f); //moves us to thumb position
    			glRotatef(-30, 0, 0, 1);
    			glPushMatrix();
    		}
    		
    		
    		glColor3f(1.0f, 0.0f, 0.0f);
    		if(selectedJoint == handJoints.get(i))
        		glColor3f(1.0f, 0.0f, 1.0f);
    		Sphere sphere = new Sphere();
        	sphere.draw(0.2f, 30, 30);
        	
    		//ROTATE by joint's angle
    		glRotatef(handJoints.get(i).angle, 1, 0, 0);
    		
    		//draw next link
    		glPushMatrix();
    		glColor3f(1.0f, 1.0f, 0.0f);
    			//align it with the x axis
    		glRotatef(-90, 1, 0, 0);
    		Cylinder bone1 = new Cylinder();
    		bone1.draw(0.1f, 0.1f, handJoints.get(i).length, 30, 30);
    		glPopMatrix(); //by this point you're back to the pinky's origin
    		
    	}
    	glPopMatrix();//2
    	glPopMatrix();//last pop
    	
    }
    
    
}
