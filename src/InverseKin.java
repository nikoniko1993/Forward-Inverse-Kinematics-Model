import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
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
import org.lwjgl.opengl.GL11;

import static  org.lwjgl.opengl.GL11.*;

import org.lwjgl.util.glu.GLU;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.glu.Sphere;
import org.lwjgl.util.glu.Cylinder;
import org.ejml.factory.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;
import org.newdawn.slick.opengl.Texture;
import org.newdawn.slick.opengl.TextureLoader;
import org.newdawn.slick.util.ResourceLoader;

public class InverseKin {

    String windowTitle = "INVERSE Kinematics";
    public boolean closeRequested = false;

    long lastFrameTime; // used to calculate delta
    
    float triangleAngle; // Angle of rotation for the triangles
    float quadAngle; // Angle of rotation for the quads
    
    public static Joint selectedJoint;
    public static ArrayList<Joint> robotArmJoints = new ArrayList<Joint>();
    public static ArrayList<Joint> handJoints = new ArrayList<Joint>();
    public static ArrayList<Joint> robotArmJoints2 = new ArrayList<Joint>();
    public static ArrayList<Joint> selectedModel;
    public static float [] robotAngles = {0, 0, 0, 0};
    public static float [] robotLengths = {1.5f, 1.5f, 1.5f, 1.5f};
    public static float [] robotAngles2 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static float [] robotLengths2 = {1f, 1f, 1f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f};
    

    public static SimpleMatrix currentPos = new SimpleMatrix(2, 1);
    public static SimpleMatrix targetPos = new SimpleMatrix(2, 1);
    
    //displaylists:
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
    	try
		{
			starry = TextureLoader.getTexture("JPG", ResourceLoader.getResourceAsStream("/Textures/OuterSpace/outerSpace.jpg"));
		}
		catch(IOException e)
		{
			System.out.println("MEH");
		}
    	
    	makeArmJoints();
    	makeArmJoints2();
    	//makeHandJoints();
    	selectedJoint = robotArmJoints.get(3); //by default, initial selected joint is root joint
    	selectedModel = robotArmJoints; //by default we begin with the robot arm selected
    	SimpleMatrix initAngles = new SimpleMatrix(4, 1);
    	initAngles.set(0, 0);
    	initAngles.set(1, 0);
    	initAngles.set(2, 0);
    	initAngles.set(3, 0);
    	currentPos = calcNewPos(initAngles);
    	targetPos = currentPos.copy();

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
		ambientLight.put(0.3f).put(0.3f).put(0.3f).put(1.0f).flip();
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

    private void updateLogic(int delta) {
        triangleAngle += 0.1f * delta; // Increase The Rotation Variable For The Triangles
        quadAngle -= 0.05f * delta; // Decrease The Rotation Variable For The Quads
    }


    private void renderGL() {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear The Screen And The Depth Buffer
        glLoadIdentity(); // Reset The View
        Camera.apply();
        
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
        
        
        glPushMatrix();
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
        
		if(selectedModel == robotArmJoints)
			drawArmModel();
		else
			drawArmModel2();
		
        //targetPos.set(currentPos.get(index)); 
        
        moveTo();
    }

    /**
     * Poll Input
     */
    public void pollInput() {
        Camera.acceptInput(getDelta());
        
        if (Mouse.isInsideWindow() && Mouse.isButtonDown(0))
        {
        	getTargetPos();
			//System.out.println("X = " + targetPos.get(0));
	    	//System.out.println("Y = " + targetPos.get(1));
	    	
       	}
        
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
                	selectedModel = robotArmJoints2;
                	selectedJoint = robotArmJoints2.get(9);
                }
                

            }
            else if (Keyboard.getEventKeyState() && selectedModel == robotArmJoints2)
            	if (Keyboard.getEventKey() == Keyboard.KEY_ESCAPE)
                    closeRequested = true;
                else if (Keyboard.getEventKey() == Keyboard.KEY_1)
                    selectedJoint = robotArmJoints2.get(0);
                else if (Keyboard.getEventKey() == Keyboard.KEY_2)
                    selectedJoint = robotArmJoints2.get(1);
                else if (Keyboard.getEventKey() == Keyboard.KEY_3)
                    selectedJoint = robotArmJoints2.get(2);
                else if (Keyboard.getEventKey() == Keyboard.KEY_4)
                    selectedJoint = robotArmJoints2.get(3);
                else if (Keyboard.getEventKey() == Keyboard.KEY_5)
                    selectedJoint = robotArmJoints2.get(4);
                else if (Keyboard.getEventKey() == Keyboard.KEY_6)
                    selectedJoint = robotArmJoints2.get(5);
                else if (Keyboard.getEventKey() == Keyboard.KEY_7)
                    selectedJoint = robotArmJoints2.get(6);
                else if (Keyboard.getEventKey() == Keyboard.KEY_8)
                    selectedJoint = robotArmJoints2.get(7);
                else if (Keyboard.getEventKey() == Keyboard.KEY_9)
                    selectedJoint = robotArmJoints2.get(8);
                else if (Keyboard.getEventKey() == Keyboard.KEY_0)
                    selectedJoint = robotArmJoints2.get(9);       
                else if (Keyboard.getEventKey() == Keyboard.KEY_TAB)
                {
                	selectedModel = robotArmJoints;
                	selectedJoint = robotArmJoints.get(3);
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
        new InverseKin().run();
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
    	for(int i = 0; i < robotLengths.length; i++)
    	{
    		currentJoint = new Joint(robotLengths[i], robotAngles[i]);
    		currentJoint.ancestors++; //it counts itself in its ancestor list too
    		
    		//add the joint to the Master Joint list
    		robotArmJoints.add(currentJoint);
    		
    		if((i - 1) >= 0) //if it is NOT the FIRST joint 
    		{
    			//proceed to add it as a child of the preceding Joint
    			prevJoint = robotArmJoints.get(i -1);//get parent
    			//assign Parent Joint
    			currentJoint.parentJoint = prevJoint;
    			
    			prevJoint.childrenJoint = currentJoint; //add the newly created node as its child
    			
    			Joint jointPtr = currentJoint;
    			
    			while(jointPtr.parentJoint != null)
    			{
    				jointPtr = jointPtr.parentJoint;
    				currentJoint.ancestors++;
    			}
    			
    		}
    	}
    }
    
    public static void makeArmJoints2()
    {
    	Joint prevJoint;
    	Joint currentJoint;
    	for(int i = 0; i < robotLengths2.length; i++)
    	{
    		currentJoint = new Joint(robotLengths2[i], robotAngles2[i]);
    		currentJoint.ancestors++; //it counts itself in its ancestor list too
    		//add the joint to the Master Joint list
    		robotArmJoints2.add(currentJoint);
    		
    		if((i - 1) >= 0) //if it is NOT the FIRST joint 
    		{
    			//proceed to add it as a child of the preceding Joint
    			prevJoint = robotArmJoints2.get(i -1);//get parent
    			//assign Parent Joint
    			currentJoint.parentJoint = prevJoint;
    			
    			prevJoint.childrenJoint = currentJoint; //add the newly created node as its child
    			
    			Joint jointPtr = currentJoint;
    			
    			while(jointPtr.parentJoint != null)
    			{
    				jointPtr = jointPtr.parentJoint;
    				currentJoint.ancestors++;
    			}
    			
    		}
    	}
    }

    
    public static void drawArmModel()
    {
    	//Draw BASE CIRCLE
    	glPushMatrix();	
		glRotatef(90, 1f, 0f, 0f); 
		drawCircle(1.5f);
        glPopMatrix();
        
        glPushMatrix();  
        for(int i = 0; i < robotArmJoints.size(); i++)
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
    		
    		if(selectedModel == robotArmJoints && robotArmJoints.get(i) == selectedJoint)
    		{
    			//get angles of current Configutation
    			SimpleMatrix angles = new SimpleMatrix(1, robotArmJoints.size());
    			for(int j = 0; j < angles.getNumElements(); j++ )
    			{
    				angles.set(j, robotArmJoints.get(j).angle);
    			}
    			//get new position with given angles
    			currentPos = calcNewPos(angles);
    			currentPos.print();
    			
    		}
        }
		glPopMatrix();
		
    }
    
    public static void drawArmModel2()
    {
    	//Draw BASE CIRCLE
    	glPushMatrix();	
		glRotatef(90, 1f, 0f, 0f); 
		drawCircle(1.5f);
        glPopMatrix();
        
        glPushMatrix();  
        for(int i = 0; i < robotArmJoints2.size(); i++)
        {
        	glColor3f(1.0f, 0.0f, 0.0f);
        	
        	if(selectedJoint == robotArmJoints2.get(i))
        		glColor3f(1.0f, 0.0f, 1.0f);
        	Sphere sphere = new Sphere();
        	sphere.draw(0.3f, 30, 30);
    		
    		//ROTATE by joint's angle
    		glRotatef(robotArmJoints2.get(i).angle, 0, 0, 1);
    		
    		//draw next link
    		glPushMatrix();
    		glColor3f(1.0f, 1.0f, 0.0f);
    		//align it with the x axis
    		glRotatef(90, 0, 1f, 0);
    		
    		Cylinder bone1 = new Cylinder();
    		bone1.draw(0.1f, 0.1f, robotArmJoints2.get(i).length, 30, 30);
    		glPopMatrix();
    		
    		glTranslatef(robotArmJoints2.get(i).length, 0, 0);
    		
    		//HERE i should get the position of the LAST joint I suppose?
    		if(selectedModel == robotArmJoints2 && robotArmJoints2.get(i) == selectedJoint)
    		{
    			//get the angles of the currentConfig
    			SimpleMatrix angles = new SimpleMatrix(1, robotArmJoints2.size());
    			for(int j = 0; j < angles.getNumElements(); j++ )
    			{
    				angles.set(j, robotArmJoints2.get(j).angle);
    			}
    			//calculate new position with the given angles
    			currentPos = calcNewPos(angles);
    			currentPos.print();
    			
    		}
        }
		glPopMatrix();
		
    }
    
    
    //Gets new target position from a click in the mouse
    public static void getTargetPos()
    {
    	
    	if (Mouse.isButtonDown(0)){
    		float winX = Mouse.getX();
        	float winY = Mouse.getY();
        	
        	IntBuffer viewPort = BufferUtils.createIntBuffer(16);
        	glGetInteger(GL_VIEWPORT, viewPort);
        	FloatBuffer modelviewMatrix = BufferUtils.createFloatBuffer(16);
        	glGetFloat(GL_MODELVIEW_MATRIX, modelviewMatrix);
        	FloatBuffer projectionMatrix = BufferUtils.createFloatBuffer(16);
        	glGetFloat(GL_PROJECTION_MATRIX, projectionMatrix);
        	
  	
        	FloatBuffer posXYZ = BufferUtils.createFloatBuffer(3);
        	
        	SimpleMatrix nearPoint = new SimpleMatrix(3, 1);
        	
        	GLU.gluUnProject( winX, winY, 0, modelviewMatrix, projectionMatrix, viewPort, posXYZ);
        	
        	nearPoint.set(0, posXYZ.get(0));
        	nearPoint.set(1, posXYZ.get(1));
        	nearPoint.set(2, posXYZ.get(2));

        	
        	GLU.gluUnProject( winX, winY, 1, modelviewMatrix, projectionMatrix, viewPort, posXYZ);
        	
        	SimpleMatrix farPoint = new SimpleMatrix(3, 1);
        	farPoint.set(0, posXYZ.get(0));
        	farPoint.set(1, posXYZ.get(1));
        	farPoint.set(2, posXYZ.get(2));	
        	
        	SimpleMatrix diff = farPoint.minus(nearPoint);
        	
        	float t = (float) (0 - (farPoint.get(2))/diff.get(2));
        	SimpleMatrix midPoint = farPoint.plus(diff.scale(t));     	
        	
        	targetPos.set(0, midPoint.get(0));
        	targetPos.set(1, midPoint.get(1));

    	}
    }
    
    //adjusts position of end effector to target position
    public void moveTo()
    {
    	double dist = (targetPos.minus(currentPos)).normF();
    	
    	float error = 0.1f;
    	SimpleMatrix newAngles = new SimpleMatrix(selectedModel.size(),1);
    	SimpleMatrix oldAngles = new SimpleMatrix(selectedModel.size(),1);
    	
    	if(dist > error)
    	{
        	for(int i = 0; i <selectedModel.size();i++)
        	{
        		oldAngles.set(i, selectedModel.get(i).angle); //load original angles
        	}
    		try{
        		
    			int cols = selectedJoint.ancestors;
    			
    			//the Jacobian should only be (selectedJoint + #children)*(2) dimensional
            	SimpleMatrix Jacobian = makeJacobian(cols);
            	SimpleMatrix JacTranspose = Jacobian.transpose();
            	SimpleMatrix pseudoinverse = Jacobian.mult(JacTranspose);
            	
            	SimpleMatrix scaledIdentity = SimpleMatrix.identity(2);
            	float lambda = 0.5f;
            	scaledIdentity.scale(Math.pow(lambda, 2));
            	
            	SimpleMatrix deltaTheta;
            	SimpleMatrix cray = ((pseudoinverse).plus(scaledIdentity)).invert();
            	SimpleMatrix e = targetPos.minus(currentPos);
            	deltaTheta = JacTranspose.mult(cray).mult(e);
            	newAngles = oldAngles.copy();
            	for(int i = 0; i < deltaTheta.getNumElements(); i++)
            	{
            		newAngles.set(i, (newAngles.get(i) + deltaTheta.get(i)));
            	}
            
            	for(int i = 0; i < newAngles.getNumElements(); i++)
            	{
            		//modify the angles for the drawing
            		selectedModel.get(i).angle = (float) newAngles.get(i);
            	}
            	currentPos = calcNewPos(newAngles);
        	} catch ( SingularMatrixException e ) {
                throw new IllegalArgumentException("Singular matrix");
            }
    	}

    }
    
    //Creates a Jacobian Matrix
    public static SimpleMatrix makeJacobian(int cols)  
    {
    	SimpleMatrix Jacobian = new SimpleMatrix(2, cols);
    	
    	//float newEntry
    	for(int i = 0; i < cols; i++)
    	{
    		int currentAngle = 0;
    		for(int j = 0; j < cols; j++)
    		{
    			currentAngle += selectedModel.get(j).angle;
    			if(j >= i)
    			{
    				//newEntry += (float) (Jacobian.get(0, i) + (selectedModel.get(j).length)*-1*Math.sin(currentAngle));
        			Jacobian.set(0, i, Jacobian.get(0, i) + (selectedModel.get(j).length)*-1*(Math.sin(Math.toRadians(currentAngle))) );
        			//newEntry += (float) (Jacobian.get(1, i) + (selectedModel.get(j).length)*-1*Math.sin(currentAngle));
        			Jacobian.set(1, i, Jacobian.get(1, i) + (selectedModel.get(j).length)*(Math.cos(Math.toRadians(currentAngle))) );
    			}
    		}
    	}

    	return Jacobian;

    }
    
    //obtains the new position of the selected end effector given a set of angles
    public static SimpleMatrix calcNewPos(SimpleMatrix newAngles)
    {
    	SimpleMatrix newPos = new SimpleMatrix(2, 1);
    	glPushMatrix();
    	glLoadIdentity();

    	if(selectedModel == robotArmJoints) //assumes first model
    	{
    		//HERE FOR LOOP IS ONLY LENGTH OF JOINT + #CHILDREN
    		for(int i = 0; i < selectedJoint.ancestors; i++)
            {
        		//ROTATE by joint's angle
        		glRotatef((float)newAngles.get(i), 0, 0, 1);
        		glTranslatef(robotArmJoints.get(i).length, 0, 0);
        		
            }
    		FloatBuffer modelviewMatrix = BufferUtils.createFloatBuffer(16);
	    	glGetFloat(GL_MODELVIEW, modelviewMatrix);
	    	newPos.set(0, modelviewMatrix.get(12)); 
	    	newPos.set(1, modelviewMatrix.get(13));
    	}
    	else	//calculate position of second model
    	{
    		//HERE FOR LOOP IS ONLY LENGTH OF JOINT + #CHILDREN
    		for(int i = 0; i < selectedJoint.ancestors; i++)
            {
        		//ROTATE by joint's angle
        		glRotatef((float)newAngles.get(i), 0, 0, 1);
        		glTranslatef(robotArmJoints2.get(i).length, 0, 0);
        		
            }
    		FloatBuffer modelviewMatrix = BufferUtils.createFloatBuffer(16);
	    	glGetFloat(GL_MODELVIEW, modelviewMatrix);
	    	newPos.set(0, modelviewMatrix.get(12)); 
	    	newPos.set(1, modelviewMatrix.get(13));
    	}
    	glPopMatrix();
    	return newPos;
    }
     
}	//END OF CLASS
