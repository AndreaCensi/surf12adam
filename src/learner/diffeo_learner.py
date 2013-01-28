from . import logger, np
from .diffeo_analysis import DiffeoAnalysis
from PIL import Image #@UnresolvedImport
from boot_agents.diffeo import (DiffeomorphismEstimator, diffeo_to_rgb_angle,
    diffeo_to_rgb_norm)
from boot_agents.diffeo.learning import DiffeomorphismEstimatorFaster
from boot_agents.diffeo.learning import DiffeomorphismEstimatorFFT 
from boot_agents.diffeo.learning import DiffeomorphismEstimatorRefine 
from boot_agents.diffeo.learning import DiffeomorphismEstimatorAnimation
from boot_agents.diffeo.learning import DiffeomorphismEstimatorFasterProbability
from boot_agents.diffeo.learning import DiffeomorphismEstimatorPixelized
from diffeoplan.library import DiffeoAction, DiffeoSystem
from diffeoplan.library.discdds.writing import ds_dump


class DiffeoLearner:
    ''' Organizes a list of diffeomorphism estimators to learn the diffeomorphisms '''
    
    def __init__(self, use_fast, diffeo_estimator_params):
        '''
        
        :param use_fast: Use DiffeomorphismEstimatorFaster.
        '''
        self.command_list = []
        self.state_list = []
        self.estimators = []
        self.estimators_inv = []    
        self.use_fast = use_fast
        self.diffeo_estimator_params = diffeo_estimator_params
        
    def new_estimator(self):
        if self.use_fast:
            logger.warning('Using experimental diffeo estimator')
            return DiffeomorphismEstimatorFaster(**self.diffeo_estimator_params)
        else:
            return DiffeomorphismEstimator(**self.diffeo_estimator_params)
    
    def estimator_index(self, command, state=None):
        if state is None:
            # Use original version
            return self.command_index(command)
        else:
            for i, command in enumerate(self.command_list):
                state_list = np.array(self.state_list)
#                pdb.set_trace()
                if state_list[i, 0] <= state and state <= state_list[i, 1]:
                    # Then we found the correct index
                    return i
                else:
                    # Keep looking for the estimator with the correct state interval
                    pass
                
            # If the command, state combination was not found, then initiate a 
            # new estimator
            index = len(self.command_list)
            state_interv = [state, state]
            logger.info('Adding new command %s for states in: %s' 
                        % (str(command), str(state_interv)))
            self.command_list.append(command)
            self.state_list.append(state_interv)
            self.estimators.append(self.new_estimator())
            self.estimators_inv.append(self.new_estimator())
            assert(len(self.command_list), len(self.state_list))
#            return index
            return 0
            
                    
    def command_index(self, command):
        if not command in self.command_list:    
            logger.info('Adding new command %s' % str(command))
            self.command_list.append(command)
            self.estimators.append(self.new_estimator())
            self.estimators_inv.append(self.new_estimator())
            
        index = self.command_list.index(command)
        return index
    
    def merge(self, other):
        """ Merges the values obtained by "other" with ours. 
            Note that we don't make a deep copy of structures.
        """
        for i in range(len(self.command_list)):
            # Note that they are not necessarily in the right order.
            command = self.command_list[i]
            if not command in other.command_list:
                logger.info('The other does not have %s' % str(command))
                logger.info('Ours: %s' % self.command_list)
                logger.info('His:  %s' % other.command_list)
                continue
            j = other.command_list.index(command)
            
            self.estimators[i].merge(other.estimators[j])
            self.estimators_inv[i].merge(other.estimators_inv[j])
        # Now add the ones we don't have.
        for j in range(len(other.command_list)):
            command = other.command_list[j]
            
            if len(other.state_list) <= j:
                state = 0
            else:
                state = other.state_list[j]
            
            if command in self.command_list:
                state_list = np.array(self.state_list)
#                pdb.set_trace()
                try:
                    if (state_list[self.command_list.index(command)] == state).all():
                        # Already have it
                        continue
                except:
                    pass
            logger.info('Adding command %s' % str(command))
            self.command_list.append(command)
            self.state_list.append(state)
            self.estimators.append(other.estimators[j])
            self.estimators_inv.append(other.estimators_inv[j])
        
    def update(self, Y0, U0, Y1, X0=None):
        cmd_ind = self.estimator_index(U0, None)
        #logger.info('Updating estimator %s' % str(cmd_ind))
        for ch in range(3):
            if X0 is None:
                self.estimators[cmd_ind].update(Y0[:, :, ch], Y1[:, :, ch])
                self.estimators_inv[cmd_ind].update(Y1[:, :, ch], Y0[:, :, ch])
            else:
                self.estimators[cmd_ind].update(Y0[:, :, ch], Y1[:, :, ch], U0, X0)
                self.estimators_inv[cmd_ind].update(Y1[:, :, ch], Y0[:, :, ch], U0, X0)
            
                            
    def summarize(self, prefix=''):
        """
            Summarizes all estimators
            Output:
                All summarized diffeomorphisms stored in self.diffeo_list
        """
        n = len(self.estimators)
        action_list = []
        for i in range(n):
            command = np.array(self.command_list[i])
            if len(self.state_list) <= i:
                state = 0
            else:
                state = self.state_list[i]
                
            name = prefix + str(list(command)).replace(' ', '')
            diffeo = self.estimators[i].summarize()
#            DiffeoAnalysis(self.estimators[i], name, self.estimators[i].shape,
#                           self.estimators[i].lengths).make_images()

#            self.estimators[i].summarize_continuous(prefix + str(command) + '.png')
            diffeo_inv = self.estimators_inv[i].summarize()
            name = 'Uninterpreted Diffeomorphism' + str(i)
            action = DiffeoAction(name, diffeo, diffeo_inv, command, state)
            action.update_uncertainty()
            action_list.append(action)
            
        name = 'Uninterpreted Diffeomorphism System'
        self.system = DiffeoSystem(name, action_list)
        return self.system
    
    def display(self, report): 
        for i in range(len(self.estimators)):
            logger.info('Report for %d-th action' % i)
            self.estimators[i].display(report.section('d%s' % i))
            self.estimators_inv[i].display(report.section('d%s-inv' % i))
            
        
    def analyze(self, prefix='', folder=''):
        """
            Make some analysis of all estimators
            Action: DiffeoAnalysis.make_images() for all estimators
        """
        n = len(self.estimators)
        
        for i in range(n):
            command = np.array(self.command_list[i])
            name = prefix + str(list(command)).replace(' ', '')
#            diffeo = self.estimators[i].summarize()
            DiffeoAnalysis(self.estimators[i], name, self.estimators[i].shape,
                           self.estimators[i].lengths, folder).make_images()
                
                
    def diffeo_dump(self, path, name):
        '''
            Save the summarized diffeomorphisms system to a pickle file
        
            :param path: output directory
            :param name: name of this system
        '''
        # TODO: remove this function, and remove local variable self.system
        ds_dump(self.system, path, name, "Learned")
        
            
    # TODO: remove
    def show_diffeomorphisms(self):
        for i in range(len(self.estimators)): #estr in self.estimators:
            D = self.estimators[i].summarize()
            cmd = self.command_list[i]
            save_path = '.'
            #Image.fromarray(diffeo_to_rgb_angle(D.d)).show()
            pim = Image.fromarray(diffeo_to_rgb_angle(D.d)) 
            pim.save(save_path + 'cmd' + str(cmd).replace(' ', '') + 'angle.png')
            pim = Image.fromarray(diffeo_to_rgb_norm(D.d))
            pim.save(save_path + 'cmd' + str(cmd).replace(' ', '') + 'nomr.png')
            pim = Image.fromarray((D.variance * 255).astype(np.uint8))
            pim.save(save_path + 'cmd' + str(cmd).replace(' ', '') + 'variance.png')
            #Image.fromarray(diffeo_to_rgb_angle(D.d)).save('diffeoimages/dir'+str(delta[0])+','+str(delta[1])+'_n'+str(n)+'_image_'+image_str+'_phase.png')
        #pdb.set_trace()
        
#class DiffeoLearnerState(DiffeoLearner):
#    def __init__(self, use_fast, diffeo_estimator_params):
#        '''
#        
#        :param use_fast: Use DiffeomorphismEstimatorFaster.
#        '''
#        self.command_list = []
#        self.estimators = []
#        self.estimators_inv = []    
#        self.use_fast = use_fast
#        self.diffeo_estimator_params = diffeo_estimator_params
#        
#        self.state = 0
#        self.command_state_list = []
#        
#        
#    def command_state_index(self, command, state):
#        if not (command[0] == 0 or command[1] == 0):
#            command[2] = 0 
#        if not [command, state] in self.command_state_list:    
#            logger.info('Adding new command %s' % str(command))
#            self.command_state_list.append([command, state])
#            self.estimators.append(self.new_estimator())
#            self.estimators_inv.append(self.new_estimator())
#            
#        index = self.command_state_list.index([command, self.state])
#        return index
#    
#    def update(self, Y0, U0, Y1):
#        self.state += U0[2]
#        logger.info('State is now: ' + str(self.state))
#        cmd_ind = self.command_state_index(U0, self.state)
#        #logger.info('Updating estimator %s' % str(cmd_ind))
#        for ch in range(3):
#            self.estimators[cmd_ind].update(Y0[:, :, ch], Y1[:, :, ch])
#            self.estimators_inv[cmd_ind].update(Y1[:, :, ch], Y0[:, :, ch])
#            
#    def summarize(self, prefix=''):
#        """
#            Summarizes all estimators
#            Output:
#                All summarized diffeomorphisms stored in self.diffeo_list
#        """
#        n = len(self.estimators)
#        action_list = [] 
#        
#        for i in range(n):
#            state = self.state_list[i]
#            command = np.array(self.command_list[i])
#            name = prefix + str(list(command)).replace(' ', '') + 'st' + str(state)
#            diffeo = self.estimators[i].summarize()
##            DiffeoAnalysis(self.estimators[i], name, self.estimators[i].shape,
##                           self.estimators[i].lengths).make_images()
##            pdb.set_trace()
##            self.estimators[i].summarize_continuous(prefix + str(command) + '.png')
#            diffeo_inv = self.estimators_inv[i].summarize()
#            name = 'Uninterpreted Diffeomorphism' + str(i) + 'cmd' + str(command) + 'state' + str(state)
#            action = DiffeoAction(name, diffeo, diffeo_inv, command)
#            action_list.append(action)
#            
#        name = 'Uninterpreted Diffeomorphism System'
#        self.system = DiffeoSystem(name, action_list)
#        return self.system
            
class DiffeoLearnerProbability(DiffeoLearner):
    def new_estimator(self):
        if self.use_fast:
            logger.warning('Using experimental diffeo estimator')
            return DiffeomorphismEstimatorFasterProbability(**self.diffeo_estimator_params)
        else:
            return DiffeomorphismEstimator(**self.diffeo_estimator_params)

class DiffeoLearnerFFT(DiffeoLearner):
    def new_estimator(self):
        return DiffeomorphismEstimatorFFT(**self.diffeo_estimator_params)
    
    def refine_init(self):
#        pdb.set_trace()
        for i in range(len(self.estimators)):
            self.estimators[i].refine_init()
        for i in range(len(self.estimators_inv)):
            self.estimators_inv[i].refine_init()

class DiffeoLearnerRefine(DiffeoLearner):
    def new_estimator(self):
        return DiffeomorphismEstimatorRefine(**self.diffeo_estimator_params)
    
    def refine_init(self):
#        pdb.set_trace()
        for i in range(len(self.estimators)):
            self.estimators[i].refine_init()
        for i in range(len(self.estimators_inv)):
            self.estimators_inv[i].refine_init()
            
class DiffeoLearnerPixelized(DiffeoLearner):
#    def __init__(self, diffeo_estimator_params):
#        '''
#        :param use_fast: Use DiffeomorphismEstimatorFaster.
#        '''
#        self.command_list = []
#        self.state_list = []
#        self.estimators = []
#        self.estimators_inv = []
#        self.diffeo_estimator_params = diffeo_estimator_params
#        
#        all_indicies = np.array(range(max_nsensels))
#        dist = np.random.randint(nthreads, size=max_nsensels)
#        
#        # Split the sensels randomly to estimators 
#        global sensels_indicies
#        if not 'sensels_indicies' in dir():
#            sensels_indicies = []
#            for i in range(nthreads):
#                sensels_indicies.append(all_indicies[dist == i])
        
    def new_estimator(self):
#        global sensels_indicies
        return DiffeomorphismEstimatorPixelized(sensels=self.sensels,
                                                **self.diffeo_estimator_params)
    
    def estimator_index(self, command, state=None):
        if len(self.estimators) == 0:
            self.estimators.append(self.new_estimator())
            self.estimators_inv.append(self.new_estimator())
        return 0
    
    def summarize(self):
        action_list = []
        
        self.merge()
        for i, cmd_state in enumerate(self.estimators[0].output_cmd_state):
            command = cmd_state[:3]
            state = cmd_state[3]
            prefix = 'pix'
            
            name = prefix + str(list(command)).replace(' ', '')
            diffeo = self.estimators[0].summarize(command, state)

            diffeo_inv = self.estimators_inv[0].summarize(command, state)
            name = 'Uninterpreted Diffeomorphism' + str(i)
            action = DiffeoAction(name, diffeo, diffeo_inv, command, state)
            action_list.append(action)
        name = 'Pixelized Diffeomorphism System'
        self.system = DiffeoSystem(name, action_list)
        return self.system
    
    def merge(self):
        for other in self.estimators[1:]:
            self.estimators[0].merge(other)
        for other in self.estimators_inv[1:]:
            self.estimators_inv[0].merge(other)

class DiffeoLearnerAnimation(DiffeoLearner):
    def new_estimator(self):    
        if not hasattr(self, 'last_index'):
            index = 0
            logger.info('adding first estimator')
        else:
            index = self.last_index + 1
            logger.info('adding new estimator')
            
        self.last_index = index
        return DiffeomorphismEstimatorAnimation(index=index, **self.diffeo_estimator_params)
