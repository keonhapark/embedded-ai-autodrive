"""
구조 검증 테스트입니다.

순수 함수 테스트는 호출부의 분기 구조를 보지 않습니다. 예를 들어 if/elif 체인의
머리 조건을

    if not intersection_exit_time and blue_pixels >= BLUE_PIXEL_THRESHOLD:
  → if not intersection_exit_time:

로 넓히면, intersection_exit_time 은 교차로 갈래를 거쳐야만 값이 생기는데 그
갈래가 먼저 죽으므로 항상 None 이 되어 뒤따르는 elif 가 모두 도달 불가가 됩니다.
차가 주행하지 않아도 판단 함수를 따로 검사하는 테스트는 전부 통과합니다. 이 파일은 소스를 AST 로 파싱해
그런 구조를 정적으로 검사합니다.
"""

import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# 리팩토링으로 만든 계층입니다. 분기 구조 검사는 이 파일들만 봅니다.
SOURCES = ('lane_tracer.py', 'control.py', 'hardware.py', 'main.py', 'vision.py')

# 전역 상태·속성 대입 규칙은 주행에 참여하는 모듈 전체에 적용합니다.
ALL_SOURCES = SOURCES + ('shared_state.py', 'object_detector.py')

# 단독 실행 스크립트입니다. 주행 경로에 참여하지 않고 원본의 모터 코드와 핀
# 번호를 복제해 갖고 있어 GPIO 핀 객체에 직접 대입합니다. 그래서 속성 대입
# 규칙에서 제외하고, test_검사_대상_파일이_모두_있다 가 존재만 확인합니다.
STANDALONE = ('tools/key_remote.py', 'tools/collect_object_data.py')


def parse(name):
    with open(os.path.join(ROOT, name), encoding='utf-8') as f:
        return ast.parse(f.read())


def test_검사_대상_파일이_모두_있다():
    """이름을 바꾸거나 지우면 아래 검사들이 조용히 빠져나갑니다."""
    missing = [n for n in ALL_SOURCES + STANDALONE
               if not os.path.exists(os.path.join(ROOT, n))]
    assert missing == [], f"검사 대상 파일이 없다: {missing}"


def if_chains(tree):
    """소스의 모든 if/elif 체인을 [(lineno, 조건 소스), ...] 목록으로 냅니다.

    ast 에서 elif 는 orelse 에 If 노드 하나만 있는 형태로 표현됩니다.
    """
    chains = []
    seen = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.If) or id(node) in seen:
            continue
        chain = []
        cur = node
        while True:
            seen.add(id(cur))
            chain.append((cur.lineno, ast.unparse(cur.test)))
            if len(cur.orelse) == 1 and isinstance(cur.orelse[0], ast.If):
                cur = cur.orelse[0]
            else:
                break
        if len(chain) > 1:
            chains.append(chain)
    return chains


def and_terms(condition_src):
    """'a and b and c' 를 {'a','b','c'} 로 나눕니다. and 가 아니면 조건 전체 한 개입니다."""
    node = ast.parse(condition_src, mode='eval').body
    if isinstance(node, ast.BoolOp) and isinstance(node.op, ast.And):
        return {ast.unparse(v) for v in node.values}
    return {condition_src}


def test_앞_가지가_뒷_가지를_삼키지_않는다():
    """if/elif 체인에서 뒷 가지가 도달 불가인 경우를 잡습니다.

    앞 가지의 조건 전체가 뒷 가지의 and-항으로 그대로 들어 있으면, 뒷 가지가
    참인 경우 앞 가지도 항상 참이라 뒷 가지는 절대 실행되지 않습니다.

        if A:          # A 가 참이면 여기서 끝
        elif A and B:  # 도달 불가
    """
    dead = []
    for source in SOURCES:
        for chain in if_chains(parse(source)):
            for i, (line_a, cond_a) in enumerate(chain):
                for line_b, cond_b in chain[i + 1:]:
                    if cond_a in and_terms(cond_b) and cond_a != cond_b:
                        dead.append(
                            f"{source}:{line_b} 은 도달 불가 — "
                            f"{source}:{line_a} 의 조건 '{cond_a}' 을 포함한다")
    assert not dead, "\n".join(dead)


def test_control_의_판단_함수는_모두_호출된다():
    """control.py 의 공개 함수가 어딘가에서 실제로 쓰이는지 확인합니다.

    판단 함수가 고아가 되는 것을 잡습니다. 순수 함수는 자기 테스트가 통과하므로
    호출부가 사라져도 드러나지 않습니다.

    주의: 이 검사는 "호출문이 소스에 있는가"만 봅니다. 호출문이 도달 불가한
    분기 안에 있어도 통과합니다. 판단 함수 호출은 control.select_branch 의
    if/return 안에 있어 위의 삼킴 검사 대상도 아니므로, 갈래 도달은
    test_control.py 의 test_네_갈래_모두_도달_가능하다 가 봅니다.
    """
    control = parse('control.py')
    defined = {n.name for n in control.body
               if isinstance(n, ast.FunctionDef) and not n.name.startswith('_')}

    # control 모듈을 통한 호출과, control.py 내부의 직접 호출을 모두 셉니다.
    # balance_ratios·balance_threshold·is_intersection 은 control.py 내부에서만
    # 불립니다(decide_steering / select_branch 안). 그래서 lane_tracer 가 이들을
    # 부르지 않게 되는 변화는 이 검사가 잡지 못합니다.
    called = set()
    for node in ast.walk(parse('lane_tracer.py')):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'control'):
            called.add(node.func.attr)
    for node in ast.walk(control):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            called.add(node.func.id)

    orphaned = defined - called
    assert not orphaned, f"control.py 의 함수가 호출되지 않는다: {sorted(orphaned)}"


def test_Act_계층에는_픽셀_연산이_없다():
    """lane_tracer.py 가 cv2·numpy 를 import 하지 않아야 합니다.

    Sense 를 vision.py 로 분리한 구조를 고정합니다. 주석으로만 남기면 편의상
    루프에 cv2 한 줄이 다시 들어가기 쉽습니다. flip 도 vision.upright() 를 통하게
    해서 이 규칙이 성립합니다.
    """
    tree = parse('lane_tracer.py')
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    forbidden = imported & {'cv2', 'numpy'}
    assert not forbidden, f"lane_tracer.py 가 픽셀 연산 라이브러리를 import 한다: {forbidden}"


def test_판단_계층에는_하드웨어와_픽셀_연산이_없다():
    """control.py 는 숫자만 다룹니다. cv2·numpy·gpiozero·hardware 는 모두 금지입니다."""
    tree = parse('control.py')
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    forbidden = imported & {'cv2', 'numpy', 'gpiozero', 'hardware', 'picamera2'}
    assert not forbidden, f"control.py 가 금지된 모듈을 import 한다: {forbidden}"


def test_주행_상태가_모듈_전역에_없다():
    """루프 상태는 지역 변수여야 합니다.

    정지선 판정 상태가 모듈 전역이면 같은 프로세스의 두 번째 주행이 stage=1 로
    시작해 첫 정지선을 "두 번째"로 오인하고, 테스트도 실행 순서에 의존하게 됩니다.
    """
    tree = parse('lane_tracer.py')

    # tree.body 만 보면 최상위 try/if/with 안의 대입이 빠지므로 재귀로 모읍니다.
    def top_level_assigns(body):
        for n in body:
            if isinstance(n, ast.Assign):
                yield n
            elif isinstance(n, (ast.Try, ast.If, ast.With)):
                for attr in ('body', 'orelse', 'finalbody', 'handlers'):
                    for item in getattr(n, attr, []) or []:
                        inner = item.body if isinstance(item, ast.ExceptHandler) else [item]
                        yield from top_level_assigns(inner)

    globals_assigned = {t.id for n in top_level_assigns(tree.body)
                        for t in n.targets if isinstance(t, ast.Name)}
    # 상수(대문자)와 큐 시스템은 제외합니다. 소문자 이름은 상태로 봅니다.
    state_like = {g for g in globals_assigned if not g.isupper()}
    allowed = {'recognized_signs', 'last_sign_time'}
    leaked = state_like - allowed
    assert not leaked, f"주행 상태가 모듈 전역에 있다: {sorted(leaked)}"


def test_global_문으로_상태를_바꾸는_함수가_없다():
    """global 대입은 소유자를 숨깁니다. last_sign_time 만 예외로 남아 있습니다."""
    offenders = []
    for source in ALL_SOURCES:
        for node in ast.walk(parse(source)):
            if isinstance(node, ast.Global):
                for name in node.names:
                    if name != 'last_sign_time':
                        offenders.append(f"{source}:{node.lineno} global {name}")
    assert not offenders, "\n".join(offenders)


def test_다른_모듈의_속성에_대입하는_곳이_공유_상태뿐이다():
    """다른 모듈의 속성에 대입해 상태를 바꾸는 곳을 잡습니다.

    shared_state.x = ... 같은 대입은 global 문 검사로는 잡히지 않습니다.
    shared_state 는 스레드 간 공유가 목적이라 허용하고, ALL_SOURCES 에 든 파일의
    그 밖의 모듈 속성 대입은 금지합니다.
    """
    offenders = []
    for source in ALL_SOURCES:
        for node in ast.walk(parse(source)):
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id not in ('self', 'shared_state')):
                    offenders.append(
                        f"{source}:{node.lineno} {target.value.id}.{target.attr}")
    assert not offenders, "\n".join(offenders)


def test_판단_계층은_모터를_참조하지_않는다():
    """control.py 에 motor.* 접근이 없어야 합니다. Plan 이 Act 의 상태를 읽으면 안 됩니다.

    소스 텍스트를 grep 하면 주석의 "motor" 까지 걸리므로 AST 로 실제 속성
    접근만 봅니다.
    """
    offenders = []
    for node in ast.walk(parse('control.py')):
        if (isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id in ('motor', 'hardware')):
            offenders.append(f"control.py:{node.lineno} {node.value.id}.{node.attr}")
    assert not offenders, "\n".join(offenders)
