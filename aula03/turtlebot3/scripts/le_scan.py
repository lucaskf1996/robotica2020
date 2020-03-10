#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	#print(np.array(dado.ranges).round(decimals=2))
	print(dado.ranges[0])
	if dado.ranges[0]>1.02:
		print("Maior")		
		speed = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
		velocidade_saida.publish(speed)
	elif dado.ranges[0]<1:
		print("Menor")
		speed = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
		velocidade_saida.publish(speed)

	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)




	while not rospy.is_shutdown():
		rospy.sleep(5)
		print("Oieeee")
