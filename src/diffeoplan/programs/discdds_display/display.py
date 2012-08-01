from procgraph_pil import resize


def discdds_display(report, dds, image):
    overview = 'Displaying a discrete DDS with %d actions' % len(dds.actions)
    report.text('overview', overview)
    
    for action in dds.actions:
        diffeoaction_display(report.section(action.label), action)


def diffeoaction_display(report, action):
    pass
