"""
nyurdt_state_machine.py
Author: Justin Rivera
NYU RDT's implementation of a state machine.
States are added one at a time to the machine, then can be executed from a valid starting state.
Supports functionality for nested state machines.
"""

class StateMachine:
    
    def __init__(self, machine_name: str, terminate=None, callback=None) -> None:
        """
        Constructor for a StateMachine
        :param machine_name: The name of this state machine
        :param terminate: Return value for when this machine stope executing, defaults to None
        :param callback: Function to be called when this machine stops executing, defaults to None
        """

        self.name = machine_name
        self.terminate = terminate
        self.callback = callback

        '''
        Dictionary of states
        maps state_name -> state_function
        '''
        self.states = dict()
        # Starting state
        self.start = None

    
    def add_state(self, state_name: str, state_function) -> None:
        """
        Method to add a new state to the state machine
        :param state_name: Name of the state to be added
        :param state_function: Function that will execute during this state
        :return: None
        """
        # State names must be unique
        if state_name in self.states:
            raise ValueError('[{}] State already exists: {}'.format(
                self.name, state_name
            ))
        
        # Setting a starting node
        if self.start is None: self.start = state_name
        # Adding the state
        self.states[state_name] = state_function
    

    def execute(self, state_name=None):
        """
        Method to execute this state machine
        If a state name is not given, this machine will begin executing at the first state added 
        :param state_name: Name of the state to start execution at
        :return: Returns this machines terminator value
        """
        # Select the starting state
        current = state_name or self.start

        # While we haven't reached a terminating state
        while current is not None:
            # Get the next function to call
            action = self.states.get(current, None)
            # Check if function is valid
            if action is None:
                raise ValueError('[{}] Invalid state: {}'.format(
                    self.name, current
                ))
            # Execute state
            current = action()
        
        # Call callback function and return terminating value 
        if callable(self.callback): self.callback()
        return self.terminate