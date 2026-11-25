# mux_ws — 미션 명령 우선순위 선택

`autorace_mux/core.py` 의 `INPUTS` 순서가 우선순위 (crosswalk > signal > parking > slow > objavoid > cone > lane).
`input_timeout`(0.25초) 안의 명령만 유효. 결과를 50 Hz 로 `/drive/cmd`. `/mux/active_source` 로 누가 제어 중인지 확인.
시험할 때 `disabled_sources: ['cone']` 처럼 미션을 끌 수 있고, `/mux/enable false` 로 정지.

## 빌드

```bash
cd ~/tofutofu/mux_ws && source env.sh
colcon build --symlink-install && source env.sh
```
