#!/usr/bin/env python

from cr_week6_test.srv import predict_robot_expression, predict_robot_expressionResponse
import rospy
from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# Define the conditional probability tables for each variable Size, Human Action and Human Expression
cpd_size = TabularCPD(variable='Size', variable_card=2, values=[[0.5], [0.5]], state_names={'Size': ['1','2']})
cpd_action = TabularCPD(variable='Action', variable_card=3, values=[[1.0/3], [1.0/3], [1.0/3]], state_names={'Action': ['1', '2', '3']})
cpd_expression = TabularCPD(variable='Expression', variable_card=3, values=[[1.0/3], [1.0/3], [1.0/3]], state_names={'Expression': ['1', '2', '3']})
# Define the conditional probability tables for the robot emotion
# State names H,S,N correspond to Happy,Sad and Neutral
cpd_emotion = TabularCPD(variable='M', variable_card=3,
                        evidence=['Expression', 'Action', 'Size'], evidence_card=[3, 3, 2],
                        values=[[0.8, 1.0, 0.8, 1.0, 0.6, 0.8, 0.0, 0.0, 0.0, 0.1, 0.0, 0.2, 0.7, 0.8, 0.8, 0.9, 0.6,0.7],
                                [0.2, 0.0, 0.2, 0.0, 0.2, 0.2, 0.0, 0.0, 0.1, 0.1, 0.2, 0.2, 0.3, 0.2, 0.2, 0.1, 0.2,0.2],
                                [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 1.0, 1.0, 0.9, 0.8, 0.8, 0.6, 0.0, 0.0, 0.0, 0.0, 0.2,0.1]],
                        state_names={'M': ['H', 'S', 'N'], 'Expression': ['1', '2', '3'], 'Action': ['1', '2', '3'], 'Size': ['1', '2']})

# Build the Bayesian network
model = BayesianNetwork([('Size', 'M'), ('Action', 'M'), ('Expression', 'M')])

# Add the conditional probability tables to the model
model.add_cpds(cpd_size, cpd_action, cpd_expression, cpd_emotion)

def handle_predict_robot_expression(input):
    # Query the model with the evidence given in the input
    evidence = {}
    if input.human_action != 0:
        evidence['Action'] = str(input.human_action)
    if input.human_expression != 0:
        evidence['Expression'] = str(input.human_expression)
    if input.object_size != 0:
        evidence['Size'] = str(input.object_size)

    # Create an instance of the VariableElimination class and pass the Bayesian Network model as a parameter
    infer = VariableElimination(model)
    # Query the model with the given evidence and compute the posterior probability distribution over the variable 'M'
    q = infer.query(variables=['M'], evidence=evidence)
    # Return the response
    return predict_robot_expressionResponse(p_happy=q.values[0],p_sad=q.values[1],p_neutral=q.values[2])




if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('robotExpressionPrediction')
    rospy.Service('predict_robot_expression_service', predict_robot_expression, handle_predict_robot_expression)
    rospy.spin()
