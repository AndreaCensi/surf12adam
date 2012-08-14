from conf_tools.code_desc import check_generic_code_desc
from conf_tools.checks import check_necessary


def check_valid_image_config(spec):
    check_generic_code_desc(spec, 'image')


def check_valid_image(x):
    pass # TODO


def check_valid_symdiffeo_config(spec):
    check_generic_code_desc(spec, 'symdiffeo')


def check_valid_symdiffeo(x):
    pass # TODO


def check_valid_diffeo_config(spec):
    check_generic_code_desc(spec, 'diffeo')


def check_valid_diffeo(x):
    pass # TODO


def check_valid_symdds_config(spec):
    check_generic_code_desc(spec, 'DDS') 

def check_valid_discdds_config(spec):
    # TODO
    pass


def check_valid_dds(x):
    pass # TODO

def check_valid_set(x):
    necessary = [ 
                  ('id', str),
                  ('desc', str),
                  ('algorithms', list),
                  ('testcases', list),
              ]
    check_necessary(x, necessary)

