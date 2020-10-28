#date: 2018.04.19

These packages are for tutorial of transformation-related issues in ROS. 
Follow the procedures summarized below.

1. Run 'fake_ball_in_rviz' node

- rosrun fake_ball_in_rviz fake_ball_in_rviz
- rviz
- Add -> By Topics -> select '/balls' -> OK
- change the fixed frame to 'camera_link'
- check whether balls are visualized well in rviz. 

2. Run 'static_tf_example' node

- rosrun static_tf_example static_tf_example
- Add -> By display tpye -> selct 'TF' -> OK
- check wheter two frame("world"&"camera_link") are visulized well.
- change the fixed frame to 'world'

3. Run 'compute_position_in_other_frame' node

- rosrun compute_position_in_other_frame compute_position_in_other_frame 
(*this node will receive two messages. 
the first one is transformation between "world" & "camera_link" published by 'static_tf_example' node
the second one is ball's position published by 'fake_ball_in_rviz' node
then, this node will compute positions of balls in "world" frame. 

These are basic things that you should know...

Lastly, the procedure of advanced(just a little bit...) tutorial with dynamic transformation is like below.
In your project, you can use dynamic transformation publisher for your mobile platform because it is not fixed. 

1. Terminate all the nodes activated.

2. Run 'fake_ball_in_rviz' node

3. Run 'dynamic_tf_example' node
- rosrun dynamic_tf_example dynamic_tf_example
- Add -> By display tpye -> selct 'TF' -> OK
- check wheter two frame("world"&"camera_link") are visulized well.
- you can see that 'camera_link' is moving while 'world' is fixed. 

4. Run 'compute_position_in_other_frame' node
-you can see the values in changing.

=========================
#about try-catch 

Let's say there is a program like below


int main(int argc,char** argv){<br />

	printf("program started");
	while(true){
		functionA();
	}
	printf("program finished");
	return 0;
}


If there is no problem in executing functionA, the program will operate well.
If there is a problem, then error will be shown when processing functionA, then the program will be terminated. 

Now, if you apply try-catch here, the code will be like below


int main(int argc,char** argv){<br />

	printf("program started");
	while(true){
		try{
			functionA();
		}
		catch(){
			printf("error");
		}
	}
	printf("program finished");
	return 0;
}


It is trivial that the program will operate well if there is no problem in executing functionA.
In addition, this program will not be terminated if there is a problem in executing functionA. 
This try-catch will try to execute functionA, and if there is an error then the lines in catch (in the above code, printf("error")) will be executed and the program will just going on.

Then, why this try-catch is required for subscribing tf in ROS? 

When you type 'rostopic echo /tf' in your terminal after running the node "static_tf_example" or "dynamic_tf_example", 
you can see a tf information with stamped time information when a message is published. 

Computing a transformation between different time step can cause meanigless result. For example, 

transformation between "world" at 0.1s & "camera_link" at 11.32s will be meaningless, especially when one of links is moving.
Therefore, the function that compute transformation has strict rule for this timing things. 
(in source code, you can find a line
'listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);' )
Because of this strict rule, there is a possibility of error when executing 'lookupTransform' function. Especially, it occurs often at the early stage of program execution.(you can see red-colored error line when executing 'compute_position_in_other_frame')




















