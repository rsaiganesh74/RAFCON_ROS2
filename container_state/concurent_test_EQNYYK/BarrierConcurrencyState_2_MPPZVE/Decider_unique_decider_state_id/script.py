def execute(self, inputs, outputs, gvm):
    self.logger.debug("Executing decider state")
    self.logger.debug("state-inputs: %s" % str(inputs))
    if self.get_outcome_for_state_name("pub").name == "success":
        self.logger.info("Due to pub")
        
    elif self.get_outcome_for_state_name("sub").name == "success":
        self.logger.info("Due to sub")
    return 0