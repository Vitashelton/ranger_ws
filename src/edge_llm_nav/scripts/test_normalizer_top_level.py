from edge_llm_nav.task_schema import normalize_intent_graph
def main():
    cases=[({'task':[],'explanation':'abc'},{'task':[]}),({'tasks':[],'explanation':'abc'},{'task':[]}),({'task':[],'tasks':[],'note':'abc'},{'task':[]})]
    for raw,expected in cases:
        assert normalize_intent_graph(raw)==expected, normalize_intent_graph(raw)
    print('normalizer_top_level: PASS')
if __name__=='__main__': main()
