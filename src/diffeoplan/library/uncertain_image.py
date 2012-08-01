from contracts import contract


class UncertainImage():
    
    @contract(values='array[HxWx...]', scalar_uncertainty='None|array[HxW]')
    def __init__(self, values, scalar_uncertainty=None):
        self.values = values
        self.scalar_uncertainty = scalar_uncertainty
        
    def get_values(self):
        return self.values
        
