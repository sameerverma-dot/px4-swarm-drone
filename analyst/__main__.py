"""CLI: python -m analyst inventory | search | ask | report | eval"""
from __future__ import annotations

import argparse
import json
import sys

from . import config


def cmd_inventory(a):
    from .ingest import inventory, load_mission
    mid = config.resolve_mission(a.mission)
    rows = inventory(mid)
    print(f"# Inventory: {mid}  ({config.mission_dir(mid)})\n")
    print("| file | kind | rows/lines | records | supports | columns |")
    print("|---|---|---|---|---|---|")
    for r in rows:
        cols = r["columns"] if len(r["columns"]) <= 70 else r["columns"][:67] + "..."
        print(f"| {r['file']} | {r['kind']} | {r['rows'] if r['rows'] is not None else '-'} | "
              f"{r['records']} | {r['supports']} | {cols} |")
    recs = load_mission(mid)
    kinds = {r["kind"] for r in rows}
    takeover = [x for x in recs if x.fields.get("event") == "takeover"]
    verify = [x for x in recs if x.fields.get("event") == "verify"]
    took = [x.fields.get("took_over") for x in verify]
    print(f"\nTotal records: {len(recs)} (ids unique: {len({r.id for r in recs}) == len(recs)})\n")
    print("Spec feature check:")
    print(f"  detections (hazard CSVs)   : {'yes' if 'ground_hazards' in kinds or 'drone_hazards' in kinds else 'MISSING'}")
    print(f"  final hazard map (geojson) : {'yes' if 'hazard_map' in kinds else 'MISSING'}")
    print(f"  flown tracks               : {sum(r['kind'] == 'track' for r in rows)} file(s)")
    print(f"  takeover / failure events  : survey VERIFY lines {len(verify)}, 'took over' = {took}; "
          f"other takeover-type log lines: {len(takeover)}")
    print(f"  ground truth targets       : {'yes' if 'truth' in kinds else 'MISSING (geotag error only where recoverable)'}")
    prog = config.PKG_DIR.parent / "docs" / "PROGRESS.md"
    print(f"  PROGRESS.md                : {'docs/PROGRESS.md present' if prog.exists() else 'not found'}"
          " (not ingested: it narrates many flights, not this mission)")


def cmd_search(a):
    from .ingest import load_mission
    from .retrieve import Index
    mid = config.resolve_mission(a.mission)
    idx = Index(load_mission(mid))
    for rec, score in idx.search(a.query, a.k):
        print(f"{score:6.2f}  [{rec.id}] {rec.text}")


def cmd_ask(a):
    from .qa import Analyst
    mid = config.resolve_mission(a.mission)
    an = Analyst(mid, provider=a.provider)
    ans = an.ask(a.question, top_k=a.k)
    if a.json:
        print(json.dumps(ans.to_dict(), indent=2))
        return
    print(f"Q: {a.question}")
    print(f"A: {ans.answer}")
    print(f"answerable={ans.answerable}  valid={ans.valid}  latency={ans.latency_s:.2f}s  "
          f"attempts={ans.attempts}  provider={an.provider.name}")
    if ans.error:
        print(f"error: {ans.error}")
    print("citations:")
    for cid in ans.citations:
        rec = an.index.by_id.get(cid)
        print(f"  [{cid}] {rec.text if rec else '(not a record id)'}")


def cmd_report(a):
    from .report import write_report
    mid = config.resolve_mission(a.mission)
    path, report, meta = write_report(mid, provider=a.provider)
    print(f"wrote {path}")
    print(f"narrative valid={meta['narrative_valid']}  unsupported numbers={meta['unsupported_numbers']}")


def cmd_eval(a):
    from .evaluate import run_eval
    mid = config.resolve_mission(a.mission)
    out = run_eval(mid, provider=a.provider, judge_provider=a.judge_provider, top_k=a.k,
                   limit=a.limit)
    print(open(out["table_path"]).read())


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m analyst", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add(name, fn, help_):
        p = sub.add_parser(name, help=help_)
        p.add_argument("--mission", default=None, help="mission id under analyst/data/")
        p.set_defaults(fn=fn)
        return p

    add("inventory", cmd_inventory, "list data files, columns, row counts, records")
    p = add("search", cmd_search, "BM25 retrieval only (debug)")
    p.add_argument("query")
    p.add_argument("-k", type=int, default=config.TOP_K)
    for name, fn, h in [("ask", cmd_ask, "grounded Q&A with citations"),
                        ("report", cmd_report, "structured mission report"),
                        ("eval", cmd_eval, "run the eval set")]:
        p = add(name, fn, h)
        p.add_argument("--provider", default=config.DEFAULT_PROVIDER, choices=["gemini", "groq", "mock"])
        if name == "ask":
            p.add_argument("question")
            p.add_argument("--json", action="store_true")
        if name in ("ask", "eval"):
            p.add_argument("-k", type=int, default=config.TOP_K)
        if name == "eval":
            p.add_argument("--judge-provider", default=None, choices=["gemini", "groq", "mock"],
                           help="default: same as --provider")
            p.add_argument("--limit", type=int, default=None, help="run only the first N questions")
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
