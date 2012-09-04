from . import UncertainImage, contract, logger
from bootstrapping_olympics.utils import safe_pickle_dump
from conf_tools.utils import friendly_path
import os
from diffeoplan.configuration import get_current_config
from diffeoplan.library.images.distance.distance_norm import DistanceNorm
from reprep.graphics.filter_scale import scale
from conf_tools.load_entries import write_entries


class TestCase():
    
    @contract(id_tc='str', y0=UncertainImage, y1=UncertainImage, id_discdds='str',
              true_plan='None|seq(int)')
    def __init__(self, id_tc, y0, y1, id_discdds, true_plan):
        self.id_tc = id_tc
        self.y0 = y0
        self.y1 = y1
        self.id_discdds = id_discdds
        self.true_plan = list(true_plan)


    def save(self, outdir):
        """ Creates outdir/<id_tc>.pickle and outdir/<>.yaml """
        
        filename_pickle = self.id_tc + '.tc.pickle'
        filename_yaml = self.id_tc + '.tc.yaml' 
        description = {
                       'id': self.id_tc,
                       'desc': 'Automatically generated test case',
                       'code': [
                         'diffeoplan.library.load_pickle',
                         {'file:pickle': filename_pickle}]
                       }
        
        filename_pickle = os.path.join(outdir, filename_pickle)
        filename_yaml = os.path.join(outdir, filename_yaml)
        
        logger.info('Writing to %r ' % friendly_path(filename_pickle))
        safe_pickle_dump(self, filename_pickle)
        
        # TODO: make function in conf tools
        write_entries([description], filename_yaml)
            
    def display(self, report):
        report.text('summary',
                    'Testcase: %s\nPlan: %s' % (self.id_tc, self.true_plan))
        report.data('id_tc', self.id_tc)
        report.data('id_discdds', self.id_discdds)
        report.data('true_plan', self.true_plan)
        f = report.figure(cols=4)
        f.data_rgb('y0_rgb', self.y0.get_rgb(), caption='$y_0$ (rgb)')
        f.data_rgb('y1_rgb', self.y1.get_rgb(), caption='$y_1$ (rgb)')
        d = DistanceNorm(2)
        
        field = d.error_field(self.y1, self.y0)
        f.data_rgb('e_y0_y1', scale(field),
                    caption="Discrepancy between $y_0$ and $y_1.")


        f = report.figure('prediction_model', cols=4,
                          caption="""
        This is the prediction according to the learned model.""")
        try:
            discdds = get_current_config().discdds.instance(self.id_discdds)
        except Exception as e:
            logger.exception(e)
        else:
            y1p = discdds.predict(self.y0, self.true_plan)
            f.data_rgb('y1p_rgb', y1p.get_rgb(),
                       caption="$p^\star \cdot y_0$")
            f.data_rgb('y1p_rgb_u', y1p.get_rgb_uncertain(),
                       caption="Uncertainty")

            field = d.error_field(self.y1, y1p)
            f.data_rgb('e_y1p_y1', scale(field),
                       caption="Discrepancy between $y_1$ and $p^\star \cdot y_0$.")
            


