import { html, reactive } from "https://esm.sh/@arrow-js/core"
import { Modal } from "/assets/components/modal.js"

const state = reactive({
  showResetDefaultModal: false,
  showResetStockModal: false,
  frogpilotToggleStates: [],
  filteredFrogpilotToggleStates: [],
  openpilotToggleStates: [],
  filteredOpenpilotToggleStates: [],
  showFrogpilotSection: true,
  showOpenpilotSection: true,
  loadingStates: true,
  statesError: "",
  searchQuery: "",
  lastUpdatedAt: "",
})

let fileInput = null
let toggleControlInitialized = false

function ensureFileInput() {
  if (fileInput) return fileInput

  fileInput = document.createElement("input")
  fileInput.type = "file"
  fileInput.accept = ".json"
  fileInput.style.display = "none"
  fileInput.addEventListener("change", restoreToggles)
  document.body.appendChild(fileInput)
  return fileInput
}

function formatUpdatedAt(timestamp) {
  if (!timestamp) return ""

  const date = new Date(timestamp)
  if (Number.isNaN(date.getTime())) return timestamp

  return date.toLocaleString()
}

function getToggleBadgeClass(toggle) {
  return `toggle-state-badge toggle-state-badge-${toggle.status}`
}

function getFilteredToggleStates() {
  const query = state.searchQuery.trim().toLowerCase()
  if (!query) {
    return {
      frogpilot: state.frogpilotToggleStates,
      openpilot: state.openpilotToggleStates,
    }
  }

  const matches = toggle =>
    toggle.label.toLowerCase().includes(query) ||
    toggle.key.toLowerCase().includes(query) ||
    toggle.display_value.toLowerCase().includes(query) ||
      toggle.value.toLowerCase().includes(query)

  return {
    frogpilot: state.frogpilotToggleStates.filter(matches),
    openpilot: state.openpilotToggleStates.filter(matches),
  }
}

function buildToggleStateSection(title, toggles, totalCount, visible, onToggle) {
  const fragment = document.createDocumentFragment()

  const heading = document.createElement("div")
  heading.className = "toggle-state-section-header"

  const titleLabel = document.createElement("div")
  titleLabel.className = "toggle-state-section-title"
  titleLabel.textContent = title

  const toggleButton = document.createElement("button")
  toggleButton.className = "toggle-section-button"
  toggleButton.type = "button"
  toggleButton.textContent = visible ? "Hide" : "Show"
  toggleButton.addEventListener("click", onToggle)

  heading.appendChild(titleLabel)
  heading.appendChild(toggleButton)
  fragment.appendChild(heading)

  if (!visible) {
    const summary = document.createElement("div")
    summary.className = "toggle-state-summary"
    summary.textContent = `${totalCount} toggles hidden`
    fragment.appendChild(summary)
    return fragment
  }

  const summary = document.createElement("div")
  summary.className = "toggle-state-summary"
  summary.textContent = `Showing ${toggles.length} of ${totalCount}`
  fragment.appendChild(summary)

  const list = document.createElement("div")
  list.className = "toggle-state-list"

  toggles.forEach(toggle => {
    const item = document.createElement("div")
    item.className = "toggle-state-item"

    const copy = document.createElement("div")
    copy.className = "toggle-state-copy"

    const label = document.createElement("div")
    label.className = "toggle-state-label"
    label.textContent = toggle.label

    const key = document.createElement("div")
    key.className = "toggle-state-key"
    key.textContent = toggle.key

    const value = document.createElement("div")
    value.className = "toggle-state-value"

    const badge = document.createElement("span")
    badge.className = getToggleBadgeClass(toggle)
    badge.textContent = toggle.display_value

    copy.appendChild(label)
    copy.appendChild(key)
    value.appendChild(badge)
    item.appendChild(copy)
    item.appendChild(value)
    list.appendChild(item)
  })

  fragment.appendChild(list)
  return fragment
}

function mountToggleStateList() {
  const host = document.querySelector(".toggle-state-list-host")
  if (!host) return

  const frogpilotToggles = state.filteredFrogpilotToggleStates
  const openpilotToggles = state.filteredOpenpilotToggleStates
  host.replaceChildren()

  if (state.loadingStates && state.frogpilotToggleStates.length === 0 && state.openpilotToggleStates.length === 0) {
    const empty = document.createElement("div")
    empty.className = "toggle-state-empty"
    empty.textContent = "Loading toggle states..."
    host.appendChild(empty)
    return
  }

  if (state.statesError) {
    const empty = document.createElement("div")
    empty.className = "toggle-state-empty"
    empty.textContent = state.statesError
    host.appendChild(empty)
    return
  }

  if (frogpilotToggles.length === 0 && openpilotToggles.length === 0) {
    const empty = document.createElement("div")
    empty.className = "toggle-state-empty"
    empty.textContent = "No toggles match that search."
    host.appendChild(empty)
    return
  }

  if (frogpilotToggles.length > 0 || state.frogpilotToggleStates.length > 0) {
    host.appendChild(buildToggleStateSection(
      "FrogPilot Toggles",
      frogpilotToggles,
      state.frogpilotToggleStates.length,
      state.showFrogpilotSection,
      () => {
        state.showFrogpilotSection = !state.showFrogpilotSection
        mountToggleStateList()
      }
    ))
  }

  if (openpilotToggles.length > 0 || state.openpilotToggleStates.length > 0) {
    host.appendChild(buildToggleStateSection(
      "openpilot UI Toggles",
      openpilotToggles,
      state.openpilotToggleStates.length,
      state.showOpenpilotSection,
      () => {
        state.showOpenpilotSection = !state.showOpenpilotSection
        mountToggleStateList()
      }
    ))
  }
}

function syncFilteredToggleStates() {
  const filtered = getFilteredToggleStates()
  state.filteredFrogpilotToggleStates = filtered.frogpilot
  state.filteredOpenpilotToggleStates = filtered.openpilot
  queueMicrotask(mountToggleStateList)
}

async function fetchToggleStates () {
  state.loadingStates = true
  state.statesError = ""

  try {
    const response = await fetch("/api/toggles/state")
    if (!response.ok) throw new Error(`HTTP ${response.status}`)

    const result = await response.json()
    state.frogpilotToggleStates = Array.isArray(result.frogpilot_toggles) ? result.frogpilot_toggles : []
    state.openpilotToggleStates = Array.isArray(result.openpilot_ui_toggles) ? result.openpilot_ui_toggles : []
    syncFilteredToggleStates()
    state.lastUpdatedAt = formatUpdatedAt(result.updated_at)
  } catch (error) {
    state.frogpilotToggleStates = []
    state.filteredFrogpilotToggleStates = []
    state.openpilotToggleStates = []
    state.filteredOpenpilotToggleStates = []
    state.statesError = "Couldn't load current toggle states."
  } finally {
    state.loadingStates = false
  }
}

async function backupToggles () {
  const response = await fetch("/api/toggles/backup", { method: "POST" })
  const blob = await response.blob()

  const downloadUrl = URL.createObjectURL(blob)
  const downloadLink = document.createElement("a")
  downloadLink.href = downloadUrl
  downloadLink.download = "toggle-backup.json"
  downloadLink.click()
  URL.revokeObjectURL(downloadUrl)
}

async function restoreToggles (event) {
  const uploadedFile = event.target.files[0]
  if (uploadedFile) {
    const fileContents = await uploadedFile.text()
    const toggleData = JSON.parse(fileContents)

    const response = await fetch("/api/toggles/restore", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(toggleData)
    })

    const result = await response.json()
    showSnackbar(result.message || "Toggles restored!")
    await fetchToggleStates()

    event.target.value = ""
  }
}

function confirmResetDefault () {
  state.showResetDefaultModal = true;
}

async function resetTogglesToDefault () {
  state.showResetDefaultModal = false;
  showSnackbar("Resetting toggles to their default values...");
  await new Promise(resolve => setTimeout(resolve, 3000));
  showSnackbar("Rebooting...");
  await new Promise(resolve => setTimeout(resolve, 3000));
  await fetch("/api/toggles/reset_default", { method: "POST" });
}

function confirmResetStock () {
  state.showResetStockModal = true;
}

async function resetTogglesToStock () {
  state.showResetStockModal = false;
  showSnackbar("Resetting toggles to stock openpilot values...");
  await new Promise(resolve => setTimeout(resolve, 3000));
  showSnackbar("Rebooting...");
  await new Promise(resolve => setTimeout(resolve, 3000));
  await fetch("/api/toggles/reset_stock", { method: "POST" });
}

function triggerRestorePrompt () {
  ensureFileInput().click()
}

export function ToggleControl () {
  ensureFileInput()

  if (!toggleControlInitialized) {
    toggleControlInitialized = true
    fetchToggleStates()
  } else {
    queueMicrotask(mountToggleStateList)
  }

  return html`
    <div class="toggle-control-wrapper">
      <section class="toggle-control-widget">
        <div class="toggle-control-title">Backup/Restore Toggles</div>
        <p class="toggle-control-text">
          Use the buttons below to backup or restore your toggles.
        </p>
        <button class="toggle-control-button" @click="${backupToggles}">Backup Toggles</button>
        <button class="toggle-control-button" @click="${triggerRestorePrompt}">Restore Toggles</button>
      </section>

      <section class="toggle-control-widget" style="margin-left: 1.5rem">
        <div class="toggle-control-title">Reset Toggles to Default FrogPilot/Stock openpilot</div>
        <p class="toggle-control-text">
          Reset all toggles to default FrogPilot/stock openpilot settings.
        </p>
        <button class="toggle-control-button" @click="${confirmResetDefault}">
          Reset Toggles to Default
        </button>
        <button class="toggle-control-button" @click="${confirmResetStock}">
          Reset Toggles to Stock
        </button>
      </section>

      <section class="toggle-control-widget toggle-state-widget">
        <div class="toggle-control-title">Current Toggle States</div>
        <p class="toggle-control-text">
          Snapshot of the stored FrogPilot toggle values. Use refresh after making changes.
        </p>

        <div class="toggle-state-toolbar">
          <input
            class="toggle-search-input"
            type="search"
            placeholder="Search toggles"
            value="${() => state.searchQuery}"
            @input="${event => {
              state.searchQuery = event.target.value;
              syncFilteredToggleStates();
            }}"
          >
          <button
            class="toggle-control-button toggle-refresh-button"
            @click="${fetchToggleStates}"
            disabled="${() => state.loadingStates}"
          >
            ${() => state.loadingStates ? "Refreshing..." : "Refresh States"}
          </button>
        </div>

        <div class="toggle-state-meta">
          ${() => state.lastUpdatedAt ? `Last updated: ${state.lastUpdatedAt}` : "Current snapshot"}
        </div>
        <div class="toggle-state-list-host"></div>
      </section>
    </div>
    ${() => state.showResetDefaultModal ? Modal({
        title: "Reset Toggles",
        message: "Are you sure you want to reset all toggles to their default FrogPilot values?",
        onConfirm: resetTogglesToDefault,
        onCancel: () => { state.showResetDefaultModal = false; },
        confirmText: "Reset to Default"
      }) : ""}
    ${() => state.showResetStockModal ? Modal({
        title: "Reset Toggles",
        message: "Are you sure you want to reset all toggles to stock openpilot values?",
        onConfirm: resetTogglesToStock,
        onCancel: () => { state.showResetStockModal = false; },
        confirmText: "Reset to Stock"
      }) : ""}
  `
}
