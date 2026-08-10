Scope FPGA vSwitch interrupt-latency experiment
================================================

Purpose
-------

This experiment measures the selected NVMe path from physical CQ completion
visibility to the point where the XiangShan NVMe interrupt handler observes
that completion.  It complements the existing injection-only measurement,
which measures only the host coordinator's virtual INTx write.

The experiment changes QEMU and the XiangShan kernel image.  It uses the
existing coherent-alias data path and does not require a new FPGA bitstream.
It is implemented by the ``scope-fpga-vswitch`` device and is disabled by
default.

Measured path
-------------

The measured sequence is::

  physical NVMe writes a CQE
    -> QEMU observes the CQE through coherent alias
    -> QEMU updates proxy state and asserts virtual INTx
    -> FPGA interrupt_out, PLIC, and Linux IRQ dispatch
    -> selected nvme_irq() observes a pending CQE
    -> handler stores a sequence in a coherent marker page
    -> QEMU observes the new sequence through coherent alias

The handler marker is written only after ``nvme_irq()`` confirms that the
selected completion queue is pending.  The endpoint therefore represents the
handler's completion-observation point rather than the first instruction of
the architecture-level trap entry.

Software components
-------------------

The implementation is split across these files:

* ``hw/misc/scope_fpga_vswitch.c`` connects CQ observation, INTx assertion,
  marker registration, and polling to the vSwitch manager.
* ``hw/misc/scope_fpga_vswitch_irq_latency.c`` owns sample state, coherent
  marker mapping, timestamp bounds, CSV output, and distribution summaries.
* ``hw/misc/scope_fpga_vswitch_irq_latency.h`` defines the internal API and
  marker-registration constants.
* The XiangShan kernel's ``drivers/nvme/host/pci.c`` allocates the marker page,
  registers its guest physical address, and writes the sequence in the IRQ
  handler.

Marker-page contract
--------------------

Each XiangShan NVMe instance allocates one page.  QEMU consumes its first two
little-endian dwords:

.. list-table:: Coherent marker page
   :header-rows: 1

   * - Offset
     - Field
     - Producer
     - Consumer
   * - ``0x00``
     - Magic ``0x53434d50``
     - XiangShan kernel
     - QEMU
   * - ``0x04``
     - Monotonically increasing sequence
     - Selected NVMe IRQ handler
     - QEMU

During probe, the kernel registers the page address with three reserved
configuration writes:

.. list-table:: Marker registration dwords
   :header-rows: 1

   * - Config offset
     - Value
   * - ``0xf0``
     - Guest physical address bits 31:0
   * - ``0xf4``
     - Guest physical address bits 63:32
   * - ``0xf8``
     - Commit value ``0x534d0001``

QEMU ACKs these writes without modifying the ECAM shadow.  For the selected
endpoint, it persistently maps the page at::

  bypass-coherent-alias-base + guest physical address

The measured handler path contains one sequence store and the normal memory
ordering barrier.  It does not contain a C2H packet, an ECAM round trip, or a
QEMU ACK.

Timestamp method
----------------

All timestamps use ``QEMU_CLOCK_HOST``.  QEMU brackets the two remote events:

``d0``
  Completion time of the last coherent read that did not contain a new CQE.

``d1``
  Completion time of the first coherent read that contained the new CQE.

``h0``
  Completion time of the last coherent read containing the old marker
  sequence.

``h1``
  Completion time of the first coherent read containing the new marker
  sequence.

For each sample, QEMU calculates::

  full_lower    = max(h0 - d1, 0)
  full_upper    = max(h1 - d0, 0)
  full_estimate = full_lower + (full_upper - full_lower) / 2
  uncertainty   = full_upper - full_lower

The INTx-to-handler interval is bounded similarly using the start and end of
the mailbox write that asserts INTx.  Bounds and midpoint are calculated for
each sample before percentiles are computed.

While waiting for a handler marker, the QEMU RX thread busy-polls the coherent
cacheline instead of entering its normal 20-us idle sleep.  It also skips the
diagnostic INTx status readback and high-rate INTx logging until the marker is
observed.  These changes apply only while the default-off experiment is
collecting a sample.

QEMU properties
---------------

The ``scope-fpga-vswitch`` device exposes:

``irq-latency-test=on|off``
  Enable the experiment.  Default is ``off``.

``irq-latency-backend-id=N``
  Select the NVMe backend.  Default is 0.

``irq-latency-qid=N``
  Select the completion queue.  Default is 1.

``irq-latency-samples=N``
  Number of valid samples to collect.  Default is 10000.

``irq-latency-output=PATH``
  CSV output path.  Default is ``/tmp/scope_irq_latency.csv``.

Build
-----

Build QEMU from this repository::

  ninja -C build qemu-system-x86_64

The instrumented XiangShan kernel and OpenSBI image are built in the matching
nexst workspace::

  make PRJ="target:nanhu-g:proto" FPGA_BD=nm37_vu37p ARCH=riscv phy_os.os
  make PRJ="target:nanhu-g:proto" FPGA_BD=nm37_vu37p ARCH=riscv \
       DT_TARGET=XSTop_vpcie opensbi

QEMU launch
-----------

The following example targets backend 0 and I/O CQ 1::

  sudo ./qemu-system-x86_64 \
    -machine q35 -m 128M -display none -monitor null -serial null \
    -device pcie-root-port,id=rp1,bus=pcie.0 \
    -device scope-fpga-vswitch,bus=rp1,\
backend-config=/path/to/vswitch-backends.json,\
fpga-host-bdf=0000:3b:00.0,\
xdma-user-dev=/dev/xdma0_user,\
xdma-ctrl-dev=/dev/xdma0_control,\
xdma-bypass-dev=/dev/xdma0_bypass,\
guest-ddr-base=0x80000000,guest-ddr-size=0x80000000,\
dma32-ring-size=0x10000,\
irq-latency-test=on,irq-latency-backend-id=0,irq-latency-qid=1,\
irq-latency-samples=10000,\
irq-latency-output=/path/to/irq_latency.csv

Before starting the workload, QEMU must print both::

  [SCOPE IRQ LAT] enabled ... marker_transport=coherent-alias ...
  [SCOPE IRQ LAT][MARKER_READY] guest_pa=... seq=...

DUT procedure
-------------

Select the same endpoint and queue, then enable the marker last::

  echo 0 > /sys/module/nvme/parameters/scope_irq_latency_domain
  echo 3 > /sys/module/nvme/parameters/scope_irq_latency_bus
  echo 0 > /sys/module/nvme/parameters/scope_irq_latency_device
  echo 0 > /sys/module/nvme/parameters/scope_irq_latency_function
  echo 1 > /sys/module/nvme/parameters/scope_irq_latency_qid
  echo 1 > /sys/module/nvme/parameters/scope_irq_latency_enable

Use one NVMe endpoint, direct 4-KiB reads, one job, and queue depth one.  Keep
other endpoints idle so one completion maps to one INTx assertion and one
handler marker.  On a minimal image without fio::

  readlink /sys/block/nvme0n1/device
  dd if=/dev/nvme0n1 of=/dev/null bs=4096 count=12000 iflag=direct

Disable the marker after collection::

  echo 0 > /sys/module/nvme/parameters/scope_irq_latency_enable

Output and acceptance criteria
------------------------------

The CSV contains raw timestamps plus:

* ``interrupt_lower/estimate/upper_ns``
* ``full_lower/estimate/upper_ns``
* ``full_interval_ns``
* ``cqe_visibility_window_ns``
* ``marker_visibility_window_ns``

QEMU prints the mean, P50, P90, P95, P99, P99.9, and maximum for every metric.
For a reportable run, ``overlap``, ``coalesced``, ``timeout``,
``marker_seq_jump``, and ``unbracketed_cqe`` should be zero.  Any nonzero value
must be disclosed and the affected run should normally be repeated.

A representative 10000-sample run reported a full-path midpoint estimate of
449.34 us mean, 422 us P50, 676 us P95, and 720 us P99.  The corresponding
conservative upper-bound values were 533.28 us mean, 501 us P50, 761 us P95,
and 807 us P99.  These values characterize this proxy configuration and are
not a generic local-PCIe interrupt-latency baseline.

Interpretation and limitations
------------------------------

The existing 956.81-ns result is injection-only: it measures the host
coordinator's virtual INTx write path.  This experiment includes CQ detection,
QEMU scheduling and state processing, virtual INTx delivery, FPGA/PLIC/Linux
dispatch, and handler observation.  The two numbers therefore measure
different paths and should not be compared as alternative estimates of one
event.

The midpoint is an estimate derived from coherent-read brackets.  Report it
together with the conservative upper bound or interval-width distribution.  A
cycle-exact result would require FPGA timestamps for CQ visibility, INTx
assertion, and the DUT marker in one clock domain.
