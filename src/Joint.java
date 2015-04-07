
public class Joint {
	
	//public Vector3f position;
	public float length;
	public float angle;
	//public Joint parent;
	Joint childrenJoint;
	Joint parentJoint;
	public int ancestors;
	
	
	public Joint(float length, float angle)
	{
		this.length = length;
		this.angle = angle;
		ancestors = 0;
	}
	
	public void rotateClockwise()	
	{
		angle -= 1f;
	}
	
	public void rotateCounterClockwise()
	{
		angle += 1f;
	}
}