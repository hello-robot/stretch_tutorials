# Tutorial Stretch OpenAI Chat
This tutorial introduces the API from OpenAI and explains how to implement it in Stretch to make basic movement.

## Explore the API
The OpenAI API is a sophisticated language model, design to assist users with a range of [models](https://platform.openai.com/docs/models) with different capabilities and price points, as well as the ability to fine-tune custom models. There are some key concepts to understand better what we can do, lets focus only in the text generation models and the tokens. 

OpenAI's text generation models, such as GPT-4 and GPT-3.5, have undergone extensive training to comprehend both natural and formal language. These models, like GPT-4, are capable of generating text based on given inputs, often referred to as "prompts". To effectively utilize a model like GPT-4, the process involves designing prompts which essentially serve as instructions or examples to guide the model in successfully completing a given task. GPT-4 can be applied to a wide range of tasks including content or code generation, summarization, conversation, creative writing, and more.

This text generation, process text in chunks called tokens. Tokens represent commonly ocurring sequences of characters. You can checkout the OpenAI's [tokenizer tool](https://platform.openai.com/tokenizer) to test specific strings and see how they are translated into tokens. Why are tokens so important? It's simple, because depending in the number of tokens you use, as well as the model (text generation, image or audio models), it will cost money. The good news, it's not that expensive and it's really useful, just be careful when dealing with image generation, as the cost is calculated per image, in contrast to text, which is priced per 1,000 tokens, or audio, which is billed per minute. You can take a look at the [pricing](https://openai.com/pricing) page from OpenAI for more information.

In this tutorial, we are using the GPT-3.5-turbo model, one of the newer text generation models alongside GPT-4 and GPT-4-turbo, we will use this model alongside the [Chat Completion API](https://platform.openai.com/docs/guides/text-generation/chat-completions-api), this will help us with our model,so that we can "chat" with Stretch and command some basic movements using natural language. If you want to know more about this and maybe create some applications, take a look at [this examples](https://platform.openai.com/examples).

## Chat Completion API
Before jumping into the Stretch Chat code, there are some things to know well about the [Chat Completion API](https://platform.openai.com/docs/guides/text-generation/chat-completions-api), take a look at this example from the documentation:
```python
from openai import OpenAI
client = OpenAI()

response = client.chat.completions.create(
  model="gpt-3.5-turbo",
  messages=[
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Who won the world series in 2020?"},
    {"role": "assistant", "content": "The Los Angeles Dodgers won the World Series in 2020."},
    {"role": "user", "content": "Where was it played?"}
  ]
)
```
As you can see, there are different roles in the chat completion, the system, the user and the assistant, each one with it's own content. This is the base for the Chat Completion and even the base to create your own chatbot, take a look at the roles:
1. The system: You will write direct instructions, sometimes the shorter and clearer is better, but if you have a long context for the AI to understand you need to be more specific (you'll see it in the tutorial)
2. The user: This will be your input text for the model to do something, it can be questions or even normal conversations, it depends on the context of the system as well, if you want the model to know everything about robotics and you ask something about chemistry or biotechnology it will output a message that it cannot process your request.
3. The assistant: Here you can help the model to understand what you are going to do, this can also be a pre crafted bot response, take a look at the tutorial to understand this better.

## Stretch Mobility with OpenAI
!!! note
    For your safety, put stretch in an open area when you try this tutorial.

For this tutorial, we'll guide Stretch to move around and perform actions by writing our instructions in natural language within the terminal, copy the next python code and paste it in your own folder, we are only going to use Stretch body and the OpenAI python library, if you haven't installed it yet don't worry, follow [this link](https://platform.openai.com/docs/quickstart/developer-quickstart) and read the quickstart guide, there you can create an OpenAI account and setup your API key as well, this is important and it's only yours so be careful where you save it! To install the library just write down in your terminal:
```{.bash .shell-prompt}
pip3 install --upgrade openai
```

Now going to the code:
```python
from openai import OpenAI
from stretch_body import robot
import time

client = OpenAI(api_key=("OPEN_AI_KEY")) # <---------- USE YOUR API KEY HERE

def move_forward(robot):
    robot.base.translate_by(0.2)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def move_backward(robot):
    robot.base.translate_by(-0.2)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def turn_right(robot):
    robot.base.rotate_by(-1.57)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def turn_left(robot):
    robot.base.rotate_by(1.57)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def arm_front(robot):
    robot.arm.move_by(0.1)
    robot.push_command()
    robot.arm.wait_until_at_setpoint()
    time.sleep(0.1)

def arm_back(robot):
    robot.arm.move_by(-0.1)
    robot.push_command()
    robot.arm.wait_until_at_setpoint()
    time.sleep(0.1)

def lift_up(robot):
    robot.lift.move_by(0.1)
    robot.push_command()
    robot.lift.wait_until_at_setpoint()
    time.sleep(0.1)

def lift_down(robot):
    robot.lift.move_by(-0.1)
    robot.push_command()
    robot.lift.wait_until_at_setpoint()
    time.sleep(0.1)

stretch_actions = {"move_forward" : "Move the robot forward 0.2m",
                   "move_backward": "Move the robot backward 0.2m",
                   "turn_right": "Turn the robot 90 degrees to the clockwise",
                   "turn_left": "Turn the robot 90 degrees to the counter clockwise",
                   "arm_front": "Move the arm to the front 0.1m",
                   "arm_back": "Move the arm to the back 0.1m",
                   "lift_up": "Move the lift up 0.1m",
                   "lift_down": "Move the lift down 0.1m"}

stretch_actions_fn = {"move_forward" : move_forward,
                   "move_backward": move_backward,
                   "turn_right": turn_right,
                   "turn_left": turn_left,
                   "arm_front": arm_front,
                   "arm_back": arm_back,
                   "lift_up": lift_up,
                   "lift_down": lift_down}


def chatter(input_text):
    # Populate Assistance prompt
    assistance_msg = "Here is the description for each robot motion"
    for k in stretch_actions.keys():
        assistance_msg = assistance_msg + f"\n - {k} : {stretch_actions[k]} "
    
    # Define System prompt (Personality of the system)
    system_prompt = f"Assume you are a mobile robot and you are able to receive a natural language instrunctions regarding the robot's movements. Based on undestanding the instructions return a sequence of discrete actions from this list {list(stretch_actions.keys())}. The only output must be in two parts. The first part should explain the sequence and second part should only be the list the actions seperated by comma. The first part you will explain the list of actions to follow and the reason behind it. The second one must be the list of movements that we are going to use, separate them only with the '[]'. This is not an explanation, it must be only the list of movements"

    response = client.chat.completions.create(
    model="gpt-3.5-turbo",
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "assistant", "content": assistance_msg},
        {"role": "user", "content": input_text},
    ]
    )
    response_text = response.choices[0].message.content.strip().lower()
    print(f"CHATGPT RESPONSE: {response_text}")
    return response_text

def extract_action_sequence(response_text):
    # In the response text find the list that starts and ends with []
    start_index = response_text.find("[")
    end_index = response_text.find("]", start_index)

    if start_index != -1 and end_index != -1:
        movements_str = response_text[start_index + 1:end_index]
        # Split the comma-separated movements into a list. The .strip("'\x22") is used to remove both single and double quotes from the beginning and end of each movement.
        formated_list = [movement.strip().strip("'\x22") for movement in movements_str.split(",")]
        print(formated_list)
        return formated_list
    else:
        print("List of movements not found in the response.")
        return []

def execute_robot_motions(final_motion_list):
    rb = robot.Robot()
    rb.startup()
    print("Starting to execute motions....")
    for motion_key in final_motion_list:
        print(f"Executing motion: {motion_key}")
        stretch_actions_fn[motion_key](rb)
    print("Completed Executing motions...")
    rb.stop()

def stretch_chatter(input_text):
    response = chatter(input_text)
    motion_list = extract_action_sequence(response)
    execute_robot_motions(motion_list)


while True:
    response = input("What motion can I do for you?\n")
    stretch_chatter(response)
```

### The code explained
Now let's break the code down
```python
from openai import OpenAI
from stretch_body import robot
import time

client = OpenAI(api_key=("OPEN_AI_KEY")) # <---------- USE YOUR API KEY HERE
```
You need to import openai if you are going to use the API. Import robot from Stretch body for the movement and don't forget to use your secret key, if you don't use it, it will not work.

```python
def move_forward(robot):
    robot.base.translate_by(0.2)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def move_backward(robot):
    robot.base.translate_by(-0.2)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def turn_right(robot):
    robot.base.rotate_by(-1.57)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)

def turn_left(robot):
    robot.base.rotate_by(1.57)
    robot.push_command()
    robot.base.wait_until_at_setpoint()
    time.sleep(0.1)
```
We will need to make methods for every movement, for the base we will need Stretch to move Forward, Backward, turn right 90 degrees or turn left 90 degrees. Keep in mind that rotations are measured in radians, if you wish to make adjustments, ensure to perform the necessary conversion.

```python
def arm_front(robot):
    robot.arm.move_by(0.1)
    robot.push_command()
    robot.arm.wait_until_at_setpoint()
    time.sleep(0.1)

def arm_back(robot):
    robot.arm.move_by(-0.1)
    robot.push_command()
    robot.arm.wait_until_at_setpoint()
    time.sleep(0.1)

def lift_up(robot):
    robot.lift.move_by(0.1)
    robot.push_command()
    robot.lift.wait_until_at_setpoint()
    time.sleep(0.1)

def lift_down(robot):
    robot.lift.move_by(-0.1)
    robot.push_command()
    robot.lift.wait_until_at_setpoint()
    time.sleep(0.1)
```
Now for the arm and the lift, this is different from the base, with the base we needed translations and rotations but these 2 are part from the prismatic joints so we just need the command move_by.

```python
stretch_actions = {"move_forward" : "Move the robot forward 0.2m",
                   "move_backward": "Move the robot backward 0.2m",
                   "turn_right": "Turn the robot 90 degrees to the clockwise",
                   "turn_left": "Turn the robot 90 degrees to the counter clockwise",
                   "arm_front": "Move the arm to the front 0.1m",
                   "arm_back": "Move the arm to the back 0.1m",
                   "lift_up": "Move the lift up 0.1m",
                   "lift_down": "Move the lift down 0.1m"}

stretch_actions_fn = {"move_forward" : move_forward,
                   "move_backward": move_backward,
                   "turn_right": turn_right,
                   "turn_left": turn_left,
                   "arm_front": arm_front,
                   "arm_back": arm_back,
                   "lift_up": lift_up,
                   "lift_down": lift_down}
```
We will need the Large Language Model (LLM) to know what are the actions for each movement with the description, we want an explanation from the LLM about the movement made, that's why we want this description.

```python
def chatter(input_text):
    # Populate Assistance prompt
    assistance_msg = "Here is the description for each robot motion"
    for k in stretch_actions.keys():
        assistance_msg = assistance_msg + f"\n - {k} : {stretch_actions[k]} "
    
    # Define System prompt (Personality of the system)
    system_prompt = f"Assume you are a mobile robot and you are able to receive a natural language instrunctions regarding the robot's movements. Based on undestanding the instructions return a sequence of discrete actions from this list {list(stretch_actions.keys())}. The only output must be in two parts. The first part should explain the sequence and second part should only be the list the actions seperated by comma. The first part you will explain the list of actions to follow and the reason behind it. The second one must be the list of movements that we are going to use, separate them only with the '[]'. This is not an explanation, it must be only the list of movements"

    clockwise \n - arm_front: Move the arm to the front by 0.1m \n - arm_back: Move the arm to the back by 0.1m \n - lift_up: Move the lift up by 0.1m \n - lift_down: Move the lift down by 0.1m"
    response = client.chat.completions.create(
    model="gpt-3.5-turbo",
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "assistant", "content": assistance_msg},
        {"role": "user", "content": input_text},
    ]
    )
    response_text = response.choices[0].message.content.strip().lower()
    print(f"CHATGPT RESPONSE: {response_text}")
    return response_text
```
Let's commence with the chatter method. Here we initialize the LLM by specifying the system message and the assistant message. Precision in these initializations is crucial for the correct execution of our code. For the user role, we provide input via the terminal, to print the response, we utilize `response.choices[0].message.content` as outlined in the documentation. To ensure uniformity and ease of handling, we employ the `strip()` and `lower()` methods. The `strip()` function removes any trailing whitespaces, including spaces or tabs, simultaneously, `lower()` converts the response to lowercase, for instance, if the LLM outputs "MOVE_FORWARD," we transform it into "move_forward." This way we enhance consistency in handling the model's outputs.

```python
def extract_action_sequence(response_text):
    # In the response text find the list that starts and ends with []
    start_index = response_text.find("[")
    end_index = response_text.find("]", start_index)

    if start_index != -1 and end_index != -1:
        movements_str = response_text[start_index + 1:end_index]
        # Split the comma-separated movements into a list. The .strip("'\x22") is used to remove both single and double quotes from the beginning and end of each movement.
        formated_list = [movement.strip().strip("'\x22") for movement in movements_str.split(",")]
        print(formated_list)
        return formated_list
    else:
        print("List of movements not found in the response.")
        return []
```
Going into the method of extraction, our goal is to identify the action list printed by the API. To achieve this, we search for square brackets. If the content is enclosed within these brackets at both the start and end, we recognize this as the desired list. Subsequently, we proceed to split and create a newly formatted list. If the model fails to locate the list of movements, it returns an empty list.

```python
def execute_robot_motions(final_motion_list):
    rb = robot.Robot()
    rb.startup()
    print("Starting to execute motions....")
    for motion_key in final_motion_list:
        print(f"Executing motion: {motion_key}")
        stretch_actions_fn[motion_key](rb)
    print("Completed Executing motions...")
    rb.stop()
```
To execute the robot movements we need to initialize it first, then it will execute these movements based on the provided list and then stops the robot, we need the startup and the stop for the `stretch_body` to work.

```python
def stretch_chatter(input_text):
    response = chatter(input_text)
    motion_list = extract_action_sequence(response)
    execute_robot_motions(motion_list)


while True:
    response = input("What motion can I do for you?\n")
    stretch_chatter(response)
```
The `stretch_chatter` will take the user input, this input is a natural language instruction regarding the robot movements and finaly we have the while loop, this ensures that the program keeps running, allowing us to input multiple requests without restarting the program.

Now that you know how this works let's try it! Run your code in the terminal and try to input the next instructions:
```
move forward 0.2m, turn right and move the lift up 2 times, turn left and move the arm front 1 time.
```
<p align="center">
  <img src="https://github.com/hello-robot/stretch_tutorials/blob/feature/stretch_openai_chat/stretch_body/images/stretch_openai.gif"/>
</p>

## Potential Issues
- Ocassionally, the model can skip one movement (normally the final ones), like those involving the arm or lift. For instance, if you instruct it to move up/down or front/back twice and then return to the starting position, there's a possibility it might execute only one backward movement instead of two. Unfortunately, there isn't a real solution to this, however, you can mitigate this type of error by providing more specific instructions.
- Sometimes the list of actions can appear different, what the code does is to look inside the response text and find the list that starts and ends with the square brackets ‘[]’ and then proceeds to make the movements, but it can occur a case like this:
![image](https://github.com/hello-robot/stretch_tutorials/assets/141784078/3ff79191-fbbd-496c-9b21-8e25dcb78a76)

In this particular scenario, observe that the list of movements appears as [‘arm_front”, if our code searches for the list of movements inside brackets it will not find anything, While this is a rare occurrence, it happened once. In such cases, the only thing to do is to stop the code and run it again.
- As mentioned in the tutorial, issues may arise regarding the movement of Stretch when you want to make a geometric figure for example, it can take a wrong turn and keep moving, keep in mind this when you try moving Stretch around, try having it in an open area just in case.
