![RAFCON_LOGO](/source/images/RAFCON_LOGO.png "LOGO")

[Image Source: https://rafcon.readthedocs.io/en/latest/]

This Repository is intended to showcase the usage of RAFCON state machines with ROS2.

-------------------------------------------------------------------------------------------------

RAFCON uses hierarchical state machines, featuring concurrent state execution, to represent robot programs. It ships with a graphical user interface supporting the creation of state machines and contains IDE like debugging mechanisms. Alternatively, state machines can programmatically be generated using RAFCON's API. [source : https://dlr-rm.github.io/RAFCON/]

-------------------------------------------------------------------------------------------------
We have created multiple state machines in RAFCON with ROS2. Namely, we have created:
1) A ROS2 initialization state (In 'init_node')
2) A ROS2 publisher state (In 'init_node')
3) A ROS2 subscriber state  (In 'init_node')
4) Concurrent/Container state, i.e states that execute simultaneously (In 'container_state')
5) A ROS2 tf_listener state ('navigate')
6) A ROS2 action client state ('navigate')

---------------------------------------------------------------------------------------------------

Steps to Use this repository:

1) Clone the repo

2) Open RAFCON

3) In RAFCON:

	a) Open state machine
	b) Go to the folder where you cloned the repo 
	c) Click on the individual state machine folder (e.g 'init_node') 
