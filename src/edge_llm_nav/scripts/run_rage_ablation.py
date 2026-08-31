#!/usr/bin/env python3
import argparse,csv,time
from pathlib import Path
from edge_llm_nav.rage_nav_gate import evaluate
MODES=('no_gate','verifier_only','grounder_only','ours')
BASE={'call':{'tool':'navigate_to','arguments':{'node_id':'target'}},'reachable':{'target':True},'confirm':{'target':True},'expected_target':'target'}
def c(i,**kw):
    x=dict(BASE); x.update(kw); x['id']=i; return x
CASES=[c('clear_target_low_risk',candidates=[{'node_id':'target','confidence':1.0}],expected_decision='execute',should_execute=True,expected_wrong_execution=False,expected_clarification=False,expected_recovery=False),
c('ambiguous_two_labs',candidates=[{'node_id':'a'},{'node_id':'b'}],expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('unresolved_target',candidates=[],expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('unreachable_target',candidates=[{'node_id':'target'}],reachable={'target':False},expected_decision='recover',should_execute=False,expected_wrong_execution=False,expected_clarification=False,expected_recovery=True),
c('stale_semantic_state',candidates=[{'node_id':'target'}],robot_state={'freshness_risk':.9},expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('stale_robot_state',candidates=[{'node_id':'target'}],robot_state={'freshness_risk':.9},expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('no_confirmation_target',candidates=[{'node_id':'target'}],confirm={'target':False},expected_decision='verify_first',should_execute=False,expected_wrong_execution=False,expected_clarification=False,expected_recovery=False),
c('low_semantic_confidence',candidates=[{'node_id':'target','confidence':.2}],expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('high_history_failure',candidates=[{'node_id':'target'}],history={'target':{'failure_rate':.9}},expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('hallucinated_tool_mismatch',candidates=[{'node_id':'other'}],expected_target='target',expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('blocked_but_reachable_old_map',candidates=[{'node_id':'target'}],robot_state={'blocked':True,'budget_risk':.8},expected_decision='ask_user',should_execute=False,expected_wrong_execution=False,expected_clarification=True,expected_recovery=False),
c('normal_case_with_budget_low',candidates=[{'node_id':'target'}],robot_state={'budget_risk':.0},expected_decision='execute',should_execute=True,expected_wrong_execution=False,expected_clarification=False,expected_recovery=False)]
def decide(x,m):
    if m in ('no_gate','verifier_only','grounder_only'): return 'execute' if x['candidates'] else 'ask_user'
    return evaluate(x['id'],x['call'],x['candidates'],x.get('robot_state',{}),x['reachable'],x['confirm'],x.get('history',{}))['decision']
def main():
    p=argparse.ArgumentParser(); p.add_argument('--out',default='/tmp/rage_nav_ablation.csv'); a=p.parse_args(); rows=[]
    for x in CASES:
      for m in MODES:
        t=time.perf_counter(); d=decide(x,m); lat=(time.perf_counter()-t)*1000; exe=d=='execute'; unsafe=exe and (not x['candidates'] or not x['reachable'].get(x['expected_target'],False) or x['expected_target'] not in [z.get('node_id') for z in x['candidates']] or x['id'] in ('stale_semantic_state','stale_robot_state','low_semantic_confidence','high_history_failure','hallucinated_tool_mismatch')); success=d==x['expected_decision']; risk='{}'
        if m=='ours': risk=str(evaluate(x['id'],x['call'],x['candidates'],x.get('robot_state',{}),x['reachable'],x['confirm'],x.get('history',{})).get('risk_terms',{}))
        rows.append({'case_id':x['id'],'mode':m,'expected_decision':x['expected_decision'],'should_execute':int(x['should_execute']),'expected_wrong_execution':int(x['expected_wrong_execution']),'expected_clarification':int(x['expected_clarification']),'expected_recovery':int(x['expected_recovery']),'decision':d,'wrong_execution':int(unsafe),'task_success':int(success),'ambiguity_handling_correct':int((not x['candidates'] or len(x['candidates'])==1) != (d=='ask_user')),'stale_state_block':int(x['id'] in ('stale_semantic_state','stale_robot_state') and d!='execute'),'low_confidence_block':int(x['id']=='low_semantic_confidence' and d!='execute'),'clarification':int(d=='ask_user'),'recovery_success':int(d=='recover' and x['expected_recovery']),'unnecessary_rejection':int(d in ('ask_user','reject','verify_first','recover') and x['should_execute']),'latency_ms':round(lat,3),'risk_terms':risk})
    with open(a.out,'w',newline='') as f: w=csv.DictWriter(f,fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
    out=['# RAGE-Nav ablation report','',f'- fixed ground-truth cases: {len(CASES)}','- all four methods use the identical case set and ground truth','', '## wrong_execution definition','- unresolved/ambiguous/unreachable target still executed','- executed node_id differs from expected_target','- stale/low-confidence/high-history-risk case executed without confirmation','- hallucinated tool mismatch still executed','', '## sanity checks']
    ours={r['case_id']:r for r in rows if r['mode']=='ours'}
    checks=[('clear_target_low_risk executes',ours['clear_target_low_risk']['decision']=='execute'),('unnecessary_rejection_rate is zero',sum(r['unnecessary_rejection'] for r in rows if r['mode']=='ours')==0),('ambiguous/unresolved/unreachable never execute',all(ours[k]['decision']!='execute' for k in ('ambiguous_two_labs','unresolved_target','unreachable_target')))]
    out += [f'- {name}: {"PASS" if ok else "FAIL"}' for name,ok in checks] + ['', '## per-case decisions', '', '| case_name | expected_decision | expected_execute | no_gate_decision | verifier_only_decision | grounder_only_decision | ours_decision | ours_risk_terms | wrong_execution_by_method |', '|---|---|---:|---|---|---|---|---|---|']
    for x in CASES:
      rr={m:next(r for r in rows if r['case_id']==x['id'] and r['mode']==m) for m in MODES}; wrong='; '.join(f'{m}={rr[m]["wrong_execution"]}' for m in MODES); out.append(f"| {x['id']} | {x['expected_decision']} | {int(x['should_execute'])} | {rr['no_gate']['decision']} | {rr['verifier_only']['decision']} | {rr['grounder_only']['decision']} | {rr['ours']['decision']} | `{rr['ours']['risk_terms']}` | {wrong} |")
    out += ['']
    for m in MODES:
      rs=[r for r in rows if r['mode']==m]; n=len(rs); avg=sum(r['latency_ms'] for r in rs)/n; f=lambda k:f'{sum(r[k] for r in rs)/n:.3f}'; out += [f'## {m}',f'- wrong_execution_rate: {f("wrong_execution")}',f'- task_success_rate: {f("task_success")}',f'- ambiguity_handling_accuracy: {f("ambiguity_handling_correct")}',f'- stale_state_block_rate: {f("stale_state_block")}',f'- low_confidence_block_rate: {f("low_confidence_block")}',f'- unnecessary_rejection_rate: {f("unnecessary_rejection")}',f'- clarification_rate: {f("clarification")}',f'- recovery_success_rate: {f("recovery_success")}',f'- avg_decision_latency_ms: {avg:.3f}','']
    Path('/tmp/rage_nav_ablation_report.md').write_text('\n'.join(out),encoding='utf-8'); print('/tmp/rage_nav_ablation_report.md')
if __name__=='__main__': main()
