# Manifold Readiness Handshake Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `PING`/`STAT` MOP handshake so the RC-companion shows Manifold augment readiness (app up, env healthy, latest-flight mesh present + point count, disk free) before the pilot augments — turning the ring-buffer "no mesh / exited 1" failure into an upfront banner.

**Architecture:** Two new magics on the existing 16-byte MOP frame header. The Manifold answers PING with a small JSON STAT computed by a **PSDK-free C module** (`kmzrun_status`) so the logic is unit-testable with native gcc; mesh facts use the same `mesh_binary_*.ply` glob as the Python augmenter, and `env_ok` is a bounded `import flight_planner.manifold` probe. The RC-companion adds a codec + a `StatusSession` + a banner.

**Tech Stack:** C (PSDK app, aeroscan-psdk repo at `/open_app/dev`, cross-compiled aarch64 + native gcc for tests); Kotlin/Android (aero-scan repo `rc-companion/`, MSDK V5, `org.json`, JUnit).

**Spec:** `docs/superpowers/specs/2026-05-25-manifold-service-readiness-handshake-design.md`

**Companion plan (separate):** the systemd `--user` service (spec Part A) is a sibling plan, written after this one. This plan is self-contained and testable on its own.

---

## Environments & repos

- **C tasks (Phase 1–2)** run **on the Manifold** over SSH: `ssh dji@192.168.1.55`. Sources: `/open_app/dev/src/manifold3_app/` (repo `git@github.com:ramin332/aeroscan-psdk.git`, toplevel `/open_app/dev`). Native test compile: `/usr/bin/gcc` (9.4).
- **Kotlin tasks (Phase 3)** run **on the laptop** in `rc-companion/` (this repo). JVM unit tests: `./gradlew :app:testDebugUnitTest` (if the wrapper is absent, generate once with `gradle wrapper` or run from Android Studio).
- Build of the PSDK binary uses the existing `/open_app/dev/run.sh` (cmake `file(GLOB application/*.c)` picks up new `.c` files on reconfigure — `run.sh` always reconfigures).

## File structure

| File | Repo | Responsibility |
| --- | --- | --- |
| `src/manifold3_app/kmzrun_status.h` | aeroscan-psdk | Public decl of the PSDK-free status builder |
| `src/manifold3_app/kmzrun_status.c` | aeroscan-psdk | Resolve flight, glob mesh, parse PLY vertex counts, statvfs, env probe, emit STAT JSON |
| `src/manifold3_app/test_status.c` | aeroscan-psdk | Standalone native test main (fixtures) |
| `src/manifold3_app/kmz_runner.h` | aeroscan-psdk | Add `PING`/`STAT` magics + `KMZRUN_APP_VERSION` |
| `src/manifold3_app/kmz_runner.c` | aeroscan-psdk | Dispatch `is_ping` in `HandleConnection`; call builder; send STAT |
| `scripts/ping_status.py` | aeroscan-psdk | On-device integration client (PING → print STAT) — via MSDK? No: see Task 8 |
| `rc-companion/.../mop/Constants.kt` | aero-scan | `MAGIC_PING`, `MAGIC_STAT` |
| `rc-companion/.../mop/AugmentFraming.kt` | aero-scan | `buildPingFrame`, `ManifoldStatus`, `parseStatJson` |
| `rc-companion/.../mop/StatusSession.kt` | aero-scan | Connect → PING → read STAT → parse → close (bounded) |
| `rc-companion/.../ui/HomeViewModel.kt` | aero-scan | `checkStatus()`, `BannerState` mapping |
| `rc-companion/.../ui/HomeScreen.kt` | aero-scan | Render the banner |
| `rc-companion/.../mop/AugmentFramingTest.kt` | aero-scan | Extend with PING/STAT tests |

---

## Phase 1 — PSDK-free C status module (unit-testable with native gcc)

### Task 1: Scaffold the status module + standalone test harness (failing test)

**Files:**
- Create: `/open_app/dev/src/manifold3_app/kmzrun_status.h`
- Create: `/open_app/dev/src/manifold3_app/kmzrun_status.c`
- Create: `/open_app/dev/src/manifold3_app/test_status.c`

- [ ] **Step 1: Write the header**

```c
/* kmzrun_status.h — PSDK-free augment-readiness status builder.
 * No DJI/PSDK includes so it compiles & unit-tests with native gcc. */
#ifndef KMZRUN_STATUS_H
#define KMZRUN_STATUS_H

#include <stddef.h>

/* Write a NUL-terminated JSON object describing augment readiness into `out`.
 *   blackbox_dir : e.g. "/blackbox"
 *   flight_id    : e.g. "the_latest_flight" or "flight0019"
 *   venv_python  : interpreter for the deep env probe, or NULL to skip it
 *                  (env_ok reported false, env_detail "skipped")
 *   app_version  : version string embedded verbatim
 * Returns strlen(out) on success, or -1 if the JSON would overflow `out`. */
int kmzrun_build_status_json(const char *blackbox_dir,
                             const char *flight_id,
                             const char *venv_python,
                             const char *app_version,
                             char *out, size_t outsz);

#endif /* KMZRUN_STATUS_H */
```

- [ ] **Step 2: Write a stub implementation (compiles, wrong output)**

```c
/* kmzrun_status.c */
#include "kmzrun_status.h"
#include <stdio.h>

int kmzrun_build_status_json(const char *blackbox_dir,
                             const char *flight_id,
                             const char *venv_python,
                             const char *app_version,
                             char *out, size_t outsz)
{
    (void) blackbox_dir; (void) flight_id; (void) venv_python; (void) app_version;
    int n = snprintf(out, outsz, "{}");
    return (n < 0 || (size_t) n >= outsz) ? -1 : n;
}
```

- [ ] **Step 3: Write the failing test**

```c
/* test_status.c — standalone, native gcc. Builds fixtures under a temp dir. */
#include "kmzrun_status.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static void mkdirs(const char *p) { char c[512]; snprintf(c,sizeof c,"mkdir -p '%s'",p); assert(system(c)==0); }
static void write_file(const char *p, const char *content) {
    FILE *f = fopen(p, "wb"); assert(f); fputs(content, f); fclose(f);
}

/* A minimal ASCII PLY header with `n` vertices, binary body omitted (we only
 * parse the header). */
static void write_ply(const char *path, int n) {
    char hdr[256];
    snprintf(hdr, sizeof hdr,
        "ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n", n);
    write_file(path, hdr);
}

int main(void) {
    char base[256]; snprintf(base, sizeof base, "/tmp/kmzrun_status_test.%d", (int) getpid());
    char bb[300]; snprintf(bb, sizeof bb, "%s/blackbox", base);

    /* Fixture A: flightX with two mesh chunks (10 + 15 verts) */
    char perc[400]; snprintf(perc, sizeof perc, "%s/flightX/dji_perception/1", bb);
    mkdirs(perc);
    char ply[500];
    snprintf(ply, sizeof ply, "%s/mesh_binary_0.ply", perc); write_ply(ply, 10);
    snprintf(ply, sizeof ply, "%s/mesh_binary_1.ply", perc); write_ply(ply, 15);

    char out[1024];
    int r = kmzrun_build_status_json(bb, "flightX", NULL, "0.0.0-test", out, sizeof out);
    assert(r > 0);
    printf("STAT(A): %s\n", out);
    assert(strstr(out, "\"mesh_present\":true"));
    assert(strstr(out, "\"mesh_chunks\":2"));
    assert(strstr(out, "\"n_points\":25"));
    assert(strstr(out, "\"latest_flight\":\"flightX\""));
    assert(strstr(out, "\"app_version\":\"0.0.0-test\""));

    /* Fixture B: flightEmpty perception dir, no PLYs */
    char perc2[400]; snprintf(perc2, sizeof perc2, "%s/flightEmpty/dji_perception/1", bb);
    mkdirs(perc2);
    r = kmzrun_build_status_json(bb, "flightEmpty", NULL, "0.0.0-test", out, sizeof out);
    assert(r > 0);
    printf("STAT(B): %s\n", out);
    assert(strstr(out, "\"mesh_present\":false"));
    assert(strstr(out, "\"mesh_chunks\":0"));

    char rm[400]; snprintf(rm, sizeof rm, "rm -rf '%s'", base); assert(system(rm)==0);
    printf("ALL PASS\n");
    return 0;
}
```

- [ ] **Step 4: Compile and run — verify it FAILS**

Run (on Manifold):
```bash
ssh dji@192.168.1.55 'cd /open_app/dev/src/manifold3_app && \
  gcc -Wall -O0 -g test_status.c kmzrun_status.c -o /tmp/test_status && /tmp/test_status'
```
Expected: assertion failure on `mesh_present:true` (stub emits `{}`). Exit non-zero.

- [ ] **Step 5: Commit**

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && git add src/manifold3_app/kmzrun_status.h src/manifold3_app/kmzrun_status.c src/manifold3_app/test_status.c && git commit -m "kmzrun_status: scaffold + failing fixture test"'
```

---

### Task 2: Implement mesh glob + PLY vertex parse + statvfs

**Files:**
- Modify: `/open_app/dev/src/manifold3_app/kmzrun_status.c`

- [ ] **Step 1: Replace the stub with the real builder (mesh + disk; env still stubbed)**

```c
/* kmzrun_status.c */
#include "kmzrun_status.h"

#include <glob.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>

/* Read the ASCII PLY header of `path` and return its `element vertex N` count,
 * or 0 if not found / unreadable. Reads only the header (bounded). */
static long ply_vertex_count(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    char line[256];
    long n = 0;
    int lines = 0;
    while (fgets(line, sizeof line, f) && lines++ < 60) {
        if (strncmp(line, "element vertex", 14) == 0) {
            n = strtol(line + 14, NULL, 10);
        }
        if (strncmp(line, "end_header", 10) == 0) break;
    }
    fclose(f);
    return n > 0 ? n : 0;
}

/* Glob mesh_binary_*.ply under <blackbox>/<flight_id>/dji_perception/1/.
 * Fills chunks/bytes/npoints. Returns 0 always (absence == 0 chunks). */
static void scan_mesh(const char *blackbox_dir, const char *flight_id,
                      int *chunks, long long *bytes, long long *npoints) {
    *chunks = 0; *bytes = 0; *npoints = 0;
    char pat[PATH_MAX];
    snprintf(pat, sizeof pat, "%s/%s/dji_perception/1/mesh_binary_*.ply",
             blackbox_dir, flight_id);
    glob_t g;
    if (glob(pat, 0, NULL, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; i++) {
            struct stat st;
            if (stat(g.gl_pathv[i], &st) == 0) *bytes += (long long) st.st_size;
            *npoints += ply_vertex_count(g.gl_pathv[i]);
            (*chunks)++;
        }
    }
    globfree(&g);
}

/* Resolve <blackbox>/<flight_id> and copy its basename into `out`. Falls back
 * to flight_id itself if realpath fails. */
static void resolve_flight(const char *blackbox_dir, const char *flight_id,
                           char *out, size_t outsz) {
    char joined[PATH_MAX], real[PATH_MAX];
    snprintf(joined, sizeof joined, "%s/%s", blackbox_dir, flight_id);
    const char *src = realpath(joined, real) ? real : flight_id;
    const char *base = strrchr(src, '/');
    snprintf(out, outsz, "%s", base ? base + 1 : src);
}

static double free_gb(const char *blackbox_dir) {
    struct statvfs s;
    if (statvfs(blackbox_dir, &s) != 0) return -1.0;
    return (double) s.f_bavail * (double) s.f_frsize / 1e9;
}

/* env probe added in Task 3; stub returns "skipped". */
static int probe_env(const char *venv_python, char *detail, size_t dsz) {
    (void) venv_python;
    snprintf(detail, dsz, "skipped");
    return 0; /* 0 == ok-ish for now; Task 3 makes this real */
}

int kmzrun_build_status_json(const char *blackbox_dir,
                             const char *flight_id,
                             const char *venv_python,
                             const char *app_version,
                             char *out, size_t outsz)
{
    char latest[128];
    resolve_flight(blackbox_dir, flight_id, latest, sizeof latest);

    int chunks; long long bytes, npoints;
    scan_mesh(blackbox_dir, flight_id, &chunks, &bytes, &npoints);

    double fg = free_gb(blackbox_dir);

    char env_detail[128];
    int env_rc = probe_env(venv_python, env_detail, sizeof env_detail);
    const char *env_ok = (env_rc == 1) ? "true" : "false"; /* Task 3 fixes mapping */

    int n = snprintf(out, outsz,
        "{\"app_version\":\"%s\",\"flight_id\":\"%s\",\"latest_flight\":\"%s\","
        "\"mesh_present\":%s,\"mesh_chunks\":%d,\"n_points\":%lld,\"mesh_bytes\":%lld,"
        "\"blackbox_free_gb\":%.1f,\"env_ok\":%s,\"env_detail\":\"%s\"}",
        app_version, flight_id, latest,
        chunks > 0 ? "true" : "false", chunks, npoints, bytes,
        fg, env_ok, env_detail);

    return (n < 0 || (size_t) n >= outsz) ? -1 : n;
}
```

- [ ] **Step 2: Run the test — verify mesh assertions PASS**

Run:
```bash
ssh dji@192.168.1.55 'cd /open_app/dev/src/manifold3_app && \
  gcc -Wall -O0 -g test_status.c kmzrun_status.c -o /tmp/test_status && /tmp/test_status'
```
Expected: `STAT(A)` shows `mesh_chunks:2,n_points:25,mesh_present:true`; `STAT(B)` shows `mesh_present:false`; `ALL PASS`. (env assertions not tested yet.)

- [ ] **Step 3: Smoke against real /blackbox**

Run:
```bash
ssh dji@192.168.1.55 '/tmp/test_status >/dev/null; cd /open_app/dev/src/manifold3_app && cat > /tmp/smoke.c <<EOF
#include "kmzrun_status.h"
#include <stdio.h>
int main(){char o[1024];
 kmzrun_build_status_json("/blackbox","the_latest_flight",NULL,"smoke",o,sizeof o);printf("latest: %s\n",o);
 kmzrun_build_status_json("/blackbox","flight0019",NULL,"smoke",o,sizeof o);printf("0019:   %s\n",o);
 return 0;}
EOF
gcc /tmp/smoke.c kmzrun_status.c -I. -o /tmp/smoke && /tmp/smoke'
```
Expected: `latest:` shows `mesh_present:false` (flight0048 has no mesh); `0019:` shows `mesh_present:true,mesh_chunks:25` and a large `n_points`.

- [ ] **Step 4: Commit**

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && git add src/manifold3_app/kmzrun_status.c && git commit -m "kmzrun_status: mesh glob + PLY vertex parse + statvfs"'
```

---

### Task 3: Deep env probe (fork/exec `import flight_planner.manifold`, 5 s timeout)

**Files:**
- Modify: `/open_app/dev/src/manifold3_app/kmzrun_status.c`

- [ ] **Step 1: Replace the `probe_env` stub + fix env_ok mapping**

Add includes near the top (after existing): `#include <unistd.h>`, `#include <sys/wait.h>`, `#include <signal.h>`, `#include <time.h>`, `#include <errno.h>`.

```c
/* Run `<venv_python> -c "import flight_planner.manifold"` with a 5 s bound.
 * Returns: 0 = ok, 1 = import failed (nonzero exit), 2 = timed out,
 *          3 = could not launch / skipped (venv_python NULL). Writes a short
 * human detail into `detail`. */
static int probe_env(const char *venv_python, char *detail, size_t dsz) {
    if (!venv_python || !venv_python[0]) { snprintf(detail, dsz, "skipped"); return 3; }

    pid_t pid = fork();
    if (pid < 0) { snprintf(detail, dsz, "fork failed"); return 3; }
    if (pid == 0) {
        /* child: silence stdout/stderr, exec the probe */
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);
        execl(venv_python, venv_python, "-c", "import flight_planner.manifold", (char *) NULL);
        _exit(127);
    }

    const int timeout_ms = 5000, step_ms = 100;
    int waited = 0, status = 0;
    for (;;) {
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) break;
        if (r < 0) { snprintf(detail, dsz, "waitpid error"); return 3; }
        if (waited >= timeout_ms) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
            snprintf(detail, dsz, "timed out (>%ds)", timeout_ms / 1000);
            return 2;
        }
        struct timespec ts = { 0, step_ms * 1000000L };
        nanosleep(&ts, NULL);
        waited += step_ms;
    }

    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) { snprintf(detail, dsz, "ok"); return 0; }
    int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    snprintf(detail, dsz, "import failed (exit %d)", code);
    return 1;
}
```

Then in `kmzrun_build_status_json`, fix the mapping:

```c
    char env_detail[128];
    int env_rc = probe_env(venv_python, env_detail, sizeof env_detail);
    const char *env_ok = (env_rc == 0) ? "true" : "false";
```

- [ ] **Step 2: Add env-probe assertions to `test_status.c`**

Append before the `rm -rf` cleanup in `main`:

```c
    /* Fixture C: stub "python" that exits 0 → env_ok true */
    char okpy[400]; snprintf(okpy, sizeof okpy, "%s/py_ok.sh", base);
    write_file(okpy, "#!/bin/sh\nexit 0\n");
    { char c[500]; snprintf(c,sizeof c,"chmod +x '%s'",okpy); assert(system(c)==0); }
    r = kmzrun_build_status_json(bb, "flightX", okpy, "0.0.0-test", out, sizeof out);
    printf("STAT(C): %s\n", out); assert(strstr(out, "\"env_ok\":true"));

    /* Fixture D: stub that exits 1 → env_ok false, import failed */
    char badpy[400]; snprintf(badpy, sizeof badpy, "%s/py_bad.sh", base);
    write_file(badpy, "#!/bin/sh\nexit 1\n");
    { char c[500]; snprintf(c,sizeof c,"chmod +x '%s'",badpy); assert(system(c)==0); }
    r = kmzrun_build_status_json(bb, "flightX", badpy, "0.0.0-test", out, sizeof out);
    printf("STAT(D): %s\n", out);
    assert(strstr(out, "\"env_ok\":false") && strstr(out, "import failed"));

    /* Fixture E: stub that sleeps 30s → timed out within ~5s */
    char slowpy[400]; snprintf(slowpy, sizeof slowpy, "%s/py_slow.sh", base);
    write_file(slowpy, "#!/bin/sh\nsleep 30\n");
    { char c[500]; snprintf(c,sizeof c,"chmod +x '%s'",slowpy); assert(system(c)==0); }
    r = kmzrun_build_status_json(bb, "flightX", slowpy, "0.0.0-test", out, sizeof out);
    printf("STAT(E): %s\n", out);
    assert(strstr(out, "\"env_ok\":false") && strstr(out, "timed out"));
```

- [ ] **Step 3: Run the test — verify all PASS**

Run:
```bash
ssh dji@192.168.1.55 'cd /open_app/dev/src/manifold3_app && \
  gcc -Wall -O0 -g test_status.c kmzrun_status.c -o /tmp/test_status && time /tmp/test_status'
```
Expected: STAT(C) env_ok:true, STAT(D) import failed, STAT(E) timed out; `ALL PASS`. Wall time ~5 s (the timeout fixture dominates).

- [ ] **Step 4: Verify the real env probe against the deployed venv**

Run:
```bash
ssh dji@192.168.1.55 'cd /open_app/dev/src/manifold3_app && cat > /tmp/envsmoke.c <<EOF
#include "kmzrun_status.h"
#include <stdio.h>
int main(){char o[1024];
 kmzrun_build_status_json("/blackbox","the_latest_flight",
   "/open_app/dev/miniforge3/envs/aero-scan/bin/python","envsmoke",o,sizeof o);
 printf("%s\n",o);return 0;}
EOF
gcc /tmp/envsmoke.c kmzrun_status.c -I. -o /tmp/envsmoke && time /tmp/envsmoke'
```
Expected: `env_ok:true,env_detail:"ok"` (env is healthy). Note the wall time — if `import flight_planner.manifold` (open3d) routinely exceeds ~4 s, bump the timeout in `probe_env` to 8000 ms and re-run; record the measured time in the commit message.

- [ ] **Step 5: Commit**

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && git add src/manifold3_app/kmzrun_status.c src/manifold3_app/test_status.c && git commit -m "kmzrun_status: deep env probe (import flight_planner.manifold, 5s bound)"'
```

---

## Phase 2 — Wire PING/STAT into the PSDK dispatch (on-device integration)

### Task 4: Add magics + dispatch the PING in `HandleConnection`

**Files:**
- Modify: `/open_app/dev/src/manifold3_app/kmz_runner.h` (add magics + version define)
- Modify: `/open_app/dev/src/manifold3_app/kmz_runner.c` (`#include "kmzrun_status.h"`, dispatch)

- [ ] **Step 1: Add defines to `kmz_runner.h`**

Find the block defining `KMZRUN_MAGIC_AUGM/PRVW/EXEC` (mirrored in `kmz_runner.c` near line 69) and add alongside them (in `kmz_runner.h`):

```c
#define KMZRUN_MAGIC_PING   "PING"   /* RC → Manifold: request status, body_len 0 */
#define KMZRUN_MAGIC_STAT   "STAT"   /* Manifold → RC: status JSON body          */
#define KMZRUN_APP_VERSION  "0.4.0"  /* bump on protocol-relevant changes        */
#define KMZRUN_VENV_PYTHON  "/open_app/dev/miniforge3/envs/aero-scan/bin/python"
```

(If the magics currently live only in `kmz_runner.c` as `#define`s, add the new ones there too, beside the existing trio, matching the existing location.)

- [ ] **Step 2: Add the dispatch branch in `HandleConnection`**

In `kmz_runner.c`, add `#include "kmzrun_status.h"` with the other includes. Then in `HandleConnection`, extend the magic dispatch (currently `is_augm` / `is_exec`):

```c
        int is_augm = !memcmp(hdr, KMZRUN_MAGIC_AUGM, 4);
        int is_exec = !memcmp(hdr, KMZRUN_MAGIC_EXEC, 4);
        int is_ping = !memcmp(hdr, KMZRUN_MAGIC_PING, 4);
        if (is_augm) magic_str = "AUGM";
        else if (is_exec) magic_str = "EXEC";
        else if (is_ping) magic_str = "PING";
        else { /* existing bad-magic drop */ }
```

Adjust the body-size cap to allow PING (body 0):

```c
        uint32_t maxBody = is_augm ? KMZRUN_MAX_BODY_LEN : 1024u; /* EXEC + PING are small */
```

After the body is received (PING has none; `bodyLen` must be 0 — if a PING arrives with a nonzero body, drain-and-ignore is fine since cap is 1024), handle PING **before** the augm/exec branch and `continue` (keep the connection open):

```c
        if (is_ping) {
            char json[1024];
            int jn = kmzrun_build_status_json("/blackbox", 
                          getenv("AEROSCAN_FLIGHT_ID") && getenv("AEROSCAN_FLIGHT_ID")[0]
                              ? getenv("AEROSCAN_FLIGHT_ID") : "the_latest_flight",
                          KMZRUN_VENV_PYTHON, KMZRUN_APP_VERSION, json, sizeof json);
            if (jn < 0) jn = snprintf(json, sizeof json, "{\"app_version\":\"%s\",\"env_ok\":false,\"env_detail\":\"status build overflow\"}", KMZRUN_APP_VERSION);
            uint8_t shdr[KMZRUN_HDR_LEN];
            EncodeHeader(shdr, KMZRUN_MAGIC_STAT, (uint32_t) jn);
            if (SendAll(peer, shdr, KMZRUN_HDR_LEN) == 0)
                (void) SendAll(peer, (const uint8_t *) json, (uint32_t) jn);
            USER_LOG_INFO("kmz_runner: answered PING with STAT (%d B)", jn);
            if (body) osal->Free(body);
            continue;
        }
```

(Ensure the PING branch runs after `RecvAll(body)` for nonzero bodies but before `HandleAugmFrame`/`HandleApprovalFrame`. Since PING `bodyLen` is 0, `body` is NULL — the free is a no-op.)

- [ ] **Step 3: Add `kmzrun_status.c` to the build & rebuild**

`run.sh` cmake-globs `application/*.c`. The `src/manifold3_app/*.c` files are symlinked into the PSDK `application/` dir by `scripts/setup_psdk.sh` (which `run.sh` runs). Confirm the symlink for the new file, then build:

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && bash scripts/setup_psdk.sh >/dev/null 2>&1; \
  ls -l Payload-SDK-3.16.0/samples/sample_c/platform/linux/manifold3/application/kmzrun_status.c'
```
Expected: a symlink `kmzrun_status.c -> /open_app/dev/src/manifold3_app/kmzrun_status.c`. If absent, add it the same way `setup_psdk.sh` links the others (or add a `ln -s` there), then re-run.

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && ./run.sh' &   # builds + launches; Ctrl-C / kill after "bound MOP channel 49154"
```
Expected build line: links `kmzrun_status.c`, compiles clean, prints `bound MOP channel 49154, accepting`.

- [ ] **Step 4: Commit**

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && git add src/manifold3_app/kmz_runner.c src/manifold3_app/kmz_runner.h scripts/setup_psdk.sh && git commit -m "kmz_runner: answer PING with STAT readiness frame"'
```

---

### Task 5: On-device integration test (PING → STAT round-trip)

> The MOP peer is the aircraft/RC link — there is no loopback MOP server to test against on the Manifold alone. So the integration test is **manual, with the RC connected**, OR validated end-to-end by the Phase 3 RC client (Task 9). This task documents the manual check; it has no code commit.

**Files:** none (verification only).

- [ ] **Step 1: Launch the app (props off, aircraft on, RC linked)**

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && AEROSCAN_FLIGHT_ID=the_latest_flight ./run.sh'
```
Expected: `bound MOP channel 49154, accepting`.

- [ ] **Step 2: From the RC-companion (after Phase 3) tap "Check Manifold"**

Expected log line on the Manifold: `kmz_runner: answered PING with STAT (NNN B)`. RC banner renders 🔴 `flight0048 has no mesh` (since latest flight has none) — which is the correct, useful result.

- [ ] **Step 3: Pin a flight with a mesh and re-check**

```bash
ssh dji@192.168.1.55 'cd /open_app/dev && AEROSCAN_FLIGHT_ID=flight0019 ./run.sh'
```
Expected: RC banner 🟢 `flight0019 · <N>M pts`.

---

## Phase 3 — RC-companion codec, session, and banner

### Task 6: Add PING/STAT magics to `Constants.kt`

**Files:**
- Modify: `rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/mop/Constants.kt`

- [ ] **Step 1: Add the two magics beside the existing AUGM/PRVW/EXEC**

```kotlin
    const val MAGIC_PING: String = "PING"  // RC → Manifold: request status (body 0)
    const val MAGIC_STAT: String = "STAT"  // Manifold → RC: readiness JSON

    // Status query must return fast; bound well under the augment timeout.
    const val STATUS_TIMEOUT_MS: Long = 15 * 1000L
```

- [ ] **Step 2: Commit**

```bash
cd rc-companion && git add app/src/main/kotlin/com/aeroscan/rccompanion/mop/Constants.kt
git commit -m "rc-companion: add PING/STAT magics + status timeout"
```
(If this repo's git is the parent `aero-scan` repo, run from repo root and add the full path instead.)

---

### Task 7: `ManifoldStatus` + `buildPingFrame` + `parseStatJson` (TDD)

**Files:**
- Modify: `rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/mop/AugmentFraming.kt`
- Modify (test): `rc-companion/app/src/test/kotlin/com/aeroscan/rccompanion/mop/AugmentFramingTest.kt`

- [ ] **Step 1: Write the failing test**

Append to `AugmentFramingTest`:

```kotlin
    @Test
    fun pingFrame_isHeaderOnly_withZeroBody() {
        val frame = AugmentFraming.buildPingFrame()
        assertEquals(AugmentFraming.HEADER_LEN, frame.size)
        val hdr = AugmentFraming.parseHeader(frame)
        assertEquals(MopConstants.MAGIC_PING, hdr.magic)
        assertEquals(0, hdr.bodyLen)
        assertEquals(MopConstants.FRAME_VERSION, hdr.version)
    }

    @Test
    fun parseStatJson_readsAllFields() {
        val json = """
            {"app_version":"0.4.0","flight_id":"the_latest_flight",
             "latest_flight":"flight0048","mesh_present":false,"mesh_chunks":0,
             "n_points":0,"mesh_bytes":0,"blackbox_free_gb":42.1,
             "env_ok":true,"env_detail":"ok"}
        """.trimIndent().toByteArray(Charsets.UTF_8)
        val s = AugmentFraming.parseStatJson(json)
        assertEquals("flight0048", s.latestFlight)
        assertEquals(false, s.meshPresent)
        assertEquals(0L, s.nPoints)
        assertEquals(true, s.envOk)
        assertEquals("ok", s.envDetail)
        assertEquals(42.1, s.blackboxFreeGb, 0.001)
    }
```

- [ ] **Step 2: Run — verify it FAILS (unresolved references)**

Run:
```bash
cd rc-companion && ./gradlew :app:testDebugUnitTest --tests "com.aeroscan.rccompanion.mop.AugmentFramingTest"
```
Expected: compile failure — `buildPingFrame`, `parseStatJson`, `ManifoldStatus` unresolved.

- [ ] **Step 3: Implement in `AugmentFraming.kt`**

```kotlin
    /** PING is header-only (body 0). */
    fun buildPingFrame(): ByteArray = frame(MopConstants.MAGIC_PING, ByteArray(0))

    /** Manifold readiness snapshot (STAT body). Mirrors kmzrun_status.c JSON. */
    data class ManifoldStatus(
        val appVersion: String,
        val flightId: String,
        val latestFlight: String,
        val meshPresent: Boolean,
        val meshChunks: Int,
        val nPoints: Long,
        val meshBytes: Long,
        val blackboxFreeGb: Double,
        val envOk: Boolean,
        val envDetail: String,
    )

    fun parseStatJson(body: ByteArray): ManifoldStatus {
        val o = org.json.JSONObject(String(body, Charsets.UTF_8))
        return ManifoldStatus(
            appVersion = o.optString("app_version", "?"),
            flightId = o.optString("flight_id", "?"),
            latestFlight = o.optString("latest_flight", "?"),
            meshPresent = o.optBoolean("mesh_present", false),
            meshChunks = o.optInt("mesh_chunks", 0),
            nPoints = o.optLong("n_points", 0L),
            meshBytes = o.optLong("mesh_bytes", 0L),
            blackboxFreeGb = o.optDouble("blackbox_free_gb", -1.0),
            envOk = o.optBoolean("env_ok", false),
            envDetail = o.optString("env_detail", ""),
        )
    }
```

- [ ] **Step 4: Run — verify PASS**

Run:
```bash
cd rc-companion && ./gradlew :app:testDebugUnitTest --tests "com.aeroscan.rccompanion.mop.AugmentFramingTest"
```
Expected: BUILD SUCCESSFUL, both new tests green.

- [ ] **Step 5: Commit**

```bash
cd rc-companion && git add app/src/main/kotlin/com/aeroscan/rccompanion/mop/AugmentFraming.kt app/src/test/kotlin/com/aeroscan/rccompanion/mop/AugmentFramingTest.kt
git commit -m "rc-companion: PING frame + STAT/ManifoldStatus codec (tested)"
```

---

### Task 8: `StatusSession` — connect → PING → read STAT → parse → close

**Files:**
- Create: `rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/mop/StatusSession.kt`

> Mirrors `AugmentSession`'s pipeline I/O but is **bounded** (no 10-min retry) and self-contained: opens its own pipeline on the same channel and closes it. Must not run while an `AugmentSession` holds the channel — the ViewModel (Task 9) serializes this.

- [ ] **Step 1: Implement the session**

```kotlin
package com.aeroscan.rccompanion.mop

import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.mop.PipelineDeviceType
import dji.sdk.keyvalue.value.mop.TransmissionControlType
import dji.v5.manager.mop.DataResult
import dji.v5.manager.mop.Pipeline
import dji.v5.manager.mop.PipelineManager
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * One-shot Manifold readiness query: connect → send PING → read STAT → close.
 * Bounded by [MopConstants.STATUS_TIMEOUT_MS]; any failure resolves to
 * [Result.Unreachable] rather than throwing.
 */
class StatusSession(
    private val componentIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val channelId: Int = MopConstants.AUGMENT_CHANNEL_ID,
    private val deviceType: PipelineDeviceType = PipelineDeviceType.PAYLOAD,
    private val mode: TransmissionControlType = TransmissionControlType.STABLE,
) {
    sealed class Result {
        data class Ok(val status: AugmentFraming.ManifoldStatus) : Result()
        data class Unreachable(val reason: String) : Result()
    }

    suspend fun query(deadlineMs: Long = MopConstants.STATUS_TIMEOUT_MS): Result =
        withContext(Dispatchers.IO) {
            val mgr = PipelineManager.getInstance()
            val connectErr = mgr.connectPipeline(componentIndex, channelId, deviceType, mode)
            if (connectErr != null) return@withContext Result.Unreachable("connect: $connectErr")
            val pl = mgr.pipelines[channelId]
                ?: return@withContext Result.Unreachable("pipeline missing").also { closeQuietly() }
            try {
                writeAll(pl, AugmentFraming.buildPingFrame())
                val deadline = System.currentTimeMillis() + deadlineMs
                val hdr = readExactly(pl, AugmentFraming.HEADER_LEN, deadline)
                    ?: return@withContext Result.Unreachable("no STAT header (timeout)")
                val parsed = AugmentFraming.parseHeader(hdr)
                if (parsed.magic != MopConstants.MAGIC_STAT)
                    return@withContext Result.Unreachable("expected STAT, got '${parsed.magic}'")
                if (parsed.bodyLen !in 1..AugmentFraming.MAX_BODY_LEN)
                    return@withContext Result.Unreachable("bad STAT len ${parsed.bodyLen}")
                val body = readExactly(pl, parsed.bodyLen, deadline)
                    ?: return@withContext Result.Unreachable("no STAT body (timeout)")
                Result.Ok(AugmentFraming.parseStatJson(body))
            } catch (t: Throwable) {
                Log.e(TAG, "status query failed", t)
                Result.Unreachable(t.message ?: t.javaClass.simpleName)
            } finally {
                closeQuietly()
            }
        }

    private fun writeAll(pipeline: Pipeline, data: ByteArray) {
        var sent = 0
        while (sent < data.size) {
            val chunk = minOf(MopConstants.CHUNK_SIZE, data.size - sent)
            val slice = if (sent == 0 && chunk == data.size) data else data.copyOfRange(sent, sent + chunk)
            val r: DataResult = pipeline.writeData(slice)
            if (r.error != null) error("writeData: ${r.error}")
            if (r.length <= 0) error("writeData non-positive")
            sent += r.length
        }
    }

    /** Bounded read of exactly [want] bytes; returns null on deadline. */
    private fun readExactly(pipeline: Pipeline, want: Int, deadline: Long): ByteArray? {
        val out = ByteArray(want); var got = 0
        val tmp = ByteArray(MopConstants.CHUNK_SIZE)
        while (got < want) {
            if (System.currentTimeMillis() > deadline) return null
            val cap = minOf(tmp.size, want - got)
            val buf = if (cap == tmp.size) tmp else ByteArray(cap)
            val r: DataResult = pipeline.readData(buf)
            if (r.error != null) {
                val e = r.error.toString()
                if (e.contains("CLOSE", true) || e.contains("DISCONNECT", true) || e.contains("RESET", true))
                    error("readData hard error: $e")
                continue // soft timeout — keep waiting until our own deadline
            }
            if (r.length <= 0) continue
            System.arraycopy(buf, 0, out, got, r.length); got += r.length
        }
        return out
    }

    private fun closeQuietly() {
        runCatching {
            PipelineManager.getInstance().disconnectPipeline(componentIndex, channelId, deviceType, mode)
        }
    }

    companion object { private const val TAG = "StatusSession" }
}
```

- [ ] **Step 2: Compile (no unit test — MSDK pipeline needs hardware)**

Run:
```bash
cd rc-companion && ./gradlew :app:compileDebugKotlin
```
Expected: BUILD SUCCESSFUL.

- [ ] **Step 3: Commit**

```bash
cd rc-companion && git add app/src/main/kotlin/com/aeroscan/rccompanion/mop/StatusSession.kt
git commit -m "rc-companion: StatusSession (bounded PING→STAT query)"
```

---

### Task 9: `HomeViewModel` banner state + `checkStatus()` (TDD the mapping)

**Files:**
- Modify: `rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/ui/HomeViewModel.kt`
- Create (test): `rc-companion/app/src/test/kotlin/com/aeroscan/rccompanion/ui/BannerStateTest.kt`

- [ ] **Step 1: Write the failing test for the pure mapping**

```kotlin
package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.mop.AugmentFraming.ManifoldStatus
import com.aeroscan.rccompanion.mop.StatusSession
import org.junit.Assert.assertTrue
import org.junit.Test

class BannerStateTest {
    private fun status(mesh: Boolean, env: Boolean) = ManifoldStatus(
        "0.4.0", "the_latest_flight", "flight0048", mesh, if (mesh) 25 else 0,
        if (mesh) 1_200_000 else 0, 0, 12.4, env, if (env) "ok" else "import failed (exit 1)")

    @Test fun envError_outranks_noMesh() {
        val b = bannerFor(StatusSession.Result.Ok(status(mesh = false, env = false)))
        assertTrue(b is BannerState.EnvError)
    }
    @Test fun noMesh_whenEnvOkButNoMesh() {
        val b = bannerFor(StatusSession.Result.Ok(status(mesh = false, env = true)))
        assertTrue(b is BannerState.NoMesh)
    }
    @Test fun ready_whenEnvOkAndMesh() {
        val b = bannerFor(StatusSession.Result.Ok(status(mesh = true, env = true)))
        assertTrue(b is BannerState.Ready)
    }
    @Test fun unreachable_passesThrough() {
        val b = bannerFor(StatusSession.Result.Unreachable("connect: x"))
        assertTrue(b is BannerState.Unreachable)
    }
}
```

- [ ] **Step 2: Run — verify FAIL (unresolved `BannerState`, `bannerFor`)**

Run:
```bash
cd rc-companion && ./gradlew :app:testDebugUnitTest --tests "com.aeroscan.rccompanion.ui.BannerStateTest"
```
Expected: compile failure.

- [ ] **Step 3: Add `BannerState` + `bannerFor` + `checkStatus()` to `HomeViewModel.kt`**

At file scope (top level in `HomeViewModel.kt`, outside the class):

```kotlin
sealed class BannerState {
    data object Idle : BannerState()
    data object Checking : BannerState()
    data class Ready(val label: String) : BannerState()
    data class NoMesh(val label: String) : BannerState()
    data class EnvError(val label: String) : BannerState()
    data class Unreachable(val label: String) : BannerState()
}

/** Pure mapping from a status result to a banner. EnvError outranks NoMesh. */
fun bannerFor(r: com.aeroscan.rccompanion.mop.StatusSession.Result): BannerState = when (r) {
    is com.aeroscan.rccompanion.mop.StatusSession.Result.Unreachable ->
        BannerState.Unreachable("Manifold not reachable — is the app running? (${r.reason})")
    is com.aeroscan.rccompanion.mop.StatusSession.Result.Ok -> {
        val s = r.status
        when {
            !s.envOk -> BannerState.EnvError("Augment env error: ${s.envDetail}")
            !s.meshPresent -> BannerState.NoMesh("${s.latestFlight} has no mesh — completed Smart3D run?")
            else -> {
                val pts = if (s.nPoints >= 1_000_000) "%.1fM pts".format(s.nPoints / 1e6)
                          else "${s.nPoints} pts"
                BannerState.Ready("Ready — ${s.latestFlight} · $pts · %.1f GB free".format(s.blackboxFreeGb))
            }
        }
    }
}
```

Inside the `HomeViewModel` class (it is a ViewModel — use `viewModelScope`; add the `StateFlow` + action):

```kotlin
    private val _banner = kotlinx.coroutines.flow.MutableStateFlow<BannerState>(BannerState.Idle)
    val banner: kotlinx.coroutines.flow.StateFlow<BannerState> = _banner

    fun checkStatus() {
        androidx.lifecycle.viewModelScope.launch {
            _banner.value = BannerState.Checking
            _banner.value = bannerFor(com.aeroscan.rccompanion.mop.StatusSession().query())
        }
    }
```

(Match the existing import/coroutine style in `HomeViewModel.kt`; if it already imports `viewModelScope`/`launch`/`MutableStateFlow`, drop the fully-qualified names.)

- [ ] **Step 4: Run — verify PASS**

Run:
```bash
cd rc-companion && ./gradlew :app:testDebugUnitTest --tests "com.aeroscan.rccompanion.ui.BannerStateTest"
```
Expected: BUILD SUCCESSFUL, four tests green.

- [ ] **Step 5: Commit**

```bash
cd rc-companion && git add app/src/main/kotlin/com/aeroscan/rccompanion/ui/HomeViewModel.kt app/src/test/kotlin/com/aeroscan/rccompanion/ui/BannerStateTest.kt
git commit -m "rc-companion: banner state mapping + checkStatus() (tested)"
```

---

### Task 10: Render the banner + "Check Manifold" control in `HomeScreen`

**Files:**
- Modify: `rc-companion/app/src/main/kotlin/com/aeroscan/rccompanion/ui/HomeScreen.kt`

- [ ] **Step 1: Add a banner composable driven by `vm.banner`**

Read `HomeScreen.kt` first to match its Compose style (theme, existing `collectAsState`, button patterns). Add near the top of the main column:

```kotlin
    val banner by vm.banner.collectAsState()
    val (bg, text) = when (val b = banner) {
        is BannerState.Idle -> androidx.compose.ui.graphics.Color.Gray to "Tap to check Manifold"
        is BannerState.Checking -> androidx.compose.ui.graphics.Color.Gray to "Checking Manifold…"
        is BannerState.Ready -> androidx.compose.ui.graphics.Color(0xFF2E7D32) to "🟢 ${b.label}"
        is BannerState.NoMesh -> androidx.compose.ui.graphics.Color(0xFFC62828) to "🔴 ${b.label}"
        is BannerState.EnvError -> androidx.compose.ui.graphics.Color(0xFFC62828) to "🔴 ${b.label}"
        is BannerState.Unreachable -> androidx.compose.ui.graphics.Color.DarkGray to "⚪ ${b.label}"
    }
    androidx.compose.material3.Surface(color = bg) {
        androidx.compose.material3.Text(
            text = text,
            color = androidx.compose.ui.graphics.Color.White,
            modifier = androidx.compose.foundation.layout.Modifier
                .fillMaxWidth()
                .padding(12.dp),
        )
    }
    androidx.compose.material3.Button(onClick = { vm.checkStatus() }) {
        androidx.compose.material3.Text("Check Manifold")
    }
```

Also trigger an initial check when the screen first composes:

```kotlin
    androidx.compose.runtime.LaunchedEffect(Unit) { vm.checkStatus() }
```

(Clean up the fully-qualified names to match the file's existing imports.)

- [ ] **Step 2: Build the debug APK**

Run:
```bash
cd rc-companion && ./gradlew :app:assembleDebug
```
Expected: BUILD SUCCESSFUL; `app/build/outputs/apk/debug/app-debug.apk` produced.

- [ ] **Step 3: Manual verification on the RC (with Manifold app running)**

Install (`adb install -r app/build/outputs/apk/debug/app-debug.apk`), open AeroScan RC, observe the banner. With `AEROSCAN_FLIGHT_ID=the_latest_flight` → 🔴 no-mesh (flight0048). With `flight0019` → 🟢 with point count. Kill the Manifold app → ⚪ Unreachable. Cross-check the Manifold log shows `answered PING with STAT`.

- [ ] **Step 4: Commit**

```bash
cd rc-companion && git add app/src/main/kotlin/com/aeroscan/rccompanion/ui/HomeScreen.kt
git commit -m "rc-companion: Manifold readiness banner + Check Manifold button"
```

---

## Self-review (against the spec)

- **STAT schema** (spec §Part B) → Tasks 2–4 emit every field; Task 7 parses every field. ✔
- **Mesh glob matches the augmenter** (`mesh_binary_*.ply`) → Task 2 `scan_mesh`. ✔
- **Deep env probe, 5 s bound, `import flight_planner.manifold`** → Task 3. ✔ (timeout tunable; Task 3 Step 4 measures real import time.)
- **PING/STAT additive, version unchanged, old-Manifold-drops-conn degrades to Unreachable** → Task 4 dispatch + Task 8 bounded read → Unreachable. ✔
- **Banner priority EnvError > NoMesh > Ready, plus Unreachable/Checking** → Task 9 `bannerFor` + tests. ✔
- **`mesh_present=false` on flight0048, `true` on flight0019** → Task 2 Step 3 smoke + Task 5/Task 10 manual. ✔
- **Status query can't collide with augment** → StatusSession is separate + ViewModel serializes (note in Task 8/9). ✔
- **No PSDK dep in the testable core** → `kmzrun_status.c` includes only libc; verified by native `gcc` compile (Task 1–3). ✔

Type consistency: `ManifoldStatus` field names identical across Task 7 (definition), Task 9 (test + mapping). `StatusSession.Result.{Ok,Unreachable}` consistent Tasks 8–9. `BannerState.{Idle,Checking,Ready,NoMesh,EnvError,Unreachable}` consistent Tasks 9–10. ✔

Out of scope (this plan): systemd service, coexistence test, Phase 2.3 fly trigger, visual preview, mesh harvesting — tracked in the spec's Non-goals / sibling plan.
