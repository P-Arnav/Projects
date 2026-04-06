const API = window.location.origin;

// ── DOM refs ────────────────────────────────────────────────────────────────
const uploadArea       = document.getElementById("upload-area");
const fileInput        = document.getElementById("file-input");
const uploadPlaceholder = document.getElementById("upload-placeholder");
const previewImg       = document.getElementById("preview-img");
const clearImgBtn      = document.getElementById("clear-img-btn");
const ageInput         = document.getElementById("age-input");
const genderInput      = document.getElementById("gender-input");
const viewInput        = document.getElementById("view-input");
const followupInput    = document.getElementById("followup-input");
const historyText      = document.getElementById("history-text");
const predictBtn       = document.getElementById("predict-btn");
const resultsPanel     = document.getElementById("results-panel");
const resultsContent   = document.getElementById("results-content");
const emptyState       = document.getElementById("empty-state");
const loadingOverlay   = document.getElementById("loading-overlay");
const heatmapImg       = document.getElementById("heatmap-img");
const compatFill       = document.getElementById("compat-fill");
const compatScore      = document.getElementById("compat-score");
const predictionsList  = document.getElementById("predictions-list");
const tokensList       = document.getElementById("tokens-list");

let selectedFile = null;

// ── History builder ─────────────────────────────────────────────────────────
function buildHistory() {
  const age = parseInt(ageInput.value) || 50;
  const gender = genderInput.value === "M" ? "male" : "female";
  const view = viewInput.value;
  const followup = parseInt(followupInput.value) || 0;

  let h = `Patient is a ${age}-year-old ${gender}. Chest X-ray taken in ${view} view. `;
  if (followup === 0) {
    h += "This is the initial presentation.";
  } else {
    h += `Follow-up visit number ${followup}.`;
  }
  return h;
}

function updateHistoryPreview() {
  historyText.textContent = buildHistory();
}

// Update preview on any field change
[ageInput, genderInput, viewInput, followupInput].forEach(el => {
  el.addEventListener("input", updateHistoryPreview);
  el.addEventListener("change", updateHistoryPreview);
});

// Initial preview
updateHistoryPreview();

// ── Upload handling ─────────────────────────────────────────────────────────
uploadArea.addEventListener("click", (e) => {
  if (e.target === clearImgBtn || clearImgBtn.contains(e.target)) return;
  fileInput.click();
});

uploadArea.addEventListener("dragover", e => {
  e.preventDefault();
  uploadArea.classList.add("drag-over");
});

uploadArea.addEventListener("dragleave", () => uploadArea.classList.remove("drag-over"));

uploadArea.addEventListener("drop", e => {
  e.preventDefault();
  uploadArea.classList.remove("drag-over");
  const file = e.dataTransfer.files[0];
  if (file) handleFile(file);
});

fileInput.addEventListener("change", () => {
  if (fileInput.files[0]) handleFile(fileInput.files[0]);
});

function handleFile(file) {
  if (!file.type.startsWith("image/")) return;
  selectedFile = file;

  const reader = new FileReader();
  reader.onload = e => {
    previewImg.src = e.target.result;
    previewImg.hidden = false;
    uploadPlaceholder.hidden = true;
    clearImgBtn.hidden = false;
    uploadArea.classList.add("has-image");
  };
  reader.readAsDataURL(file);
  predictBtn.disabled = false;
}

clearImgBtn.addEventListener("click", (e) => {
  e.stopPropagation();
  selectedFile = null;
  previewImg.hidden = true;
  previewImg.src = "";
  uploadPlaceholder.hidden = false;
  clearImgBtn.hidden = true;
  uploadArea.classList.remove("has-image");
  predictBtn.disabled = true;
  fileInput.value = "";
});

// ── Predict ─────────────────────────────────────────────────────────────────
predictBtn.addEventListener("click", async () => {
  if (!selectedFile) return;

  loadingOverlay.hidden = false;
  resultsContent.hidden = true;
  emptyState.hidden = true;

  const formData = new FormData();
  formData.append("file", selectedFile);
  formData.append("age", ageInput.value);
  formData.append("gender", genderInput.value);
  formData.append("view", viewInput.value);
  formData.append("followup", followupInput.value);

  try {
    const res = await fetch(`${API}/predict`, { method: "POST", body: formData });
    if (!res.ok) throw new Error(`Server error: ${res.status}`);
    const data = await res.json();
    renderResults(data);
  } catch (err) {
    alert(`Error: ${err.message}`);
  } finally {
    loadingOverlay.hidden = true;
  }
});

// ── Render results ──────────────────────────────────────────────────────────
function renderResults(data) {
  // Compatibility score
  const pct = Math.round(data.compatibility * 100);
  compatFill.style.width = `${pct}%`;
  compatScore.textContent = data.compatibility.toFixed(3);

  // Heatmap
  heatmapImg.src = `data:image/png;base64,${data.heatmap_base64}`;

  // Predictions
  predictionsList.innerHTML = "";
  data.predictions.forEach(pred => {
    const row = document.createElement("div");
    const isPositive = pred.predicted;
    const barPct = Math.round(pred.prob * 100);
    row.className = `pred-row${isPositive ? " is-positive" : ""}`;

    row.innerHTML = `
      <span class="pred-name">${pred.disease.replace(/_/g, " ")}</span>
      <div class="pred-prob-cell">
        <div class="pred-bar-track">
          <div class="pred-bar-fill ${isPositive ? "positive" : "negative"}"
               style="width: ${barPct}%"></div>
        </div>
        <span class="pred-prob">${(pred.prob * 100).toFixed(1)}%</span>
      </div>
      <span class="pred-badge ${isPositive ? "yes" : "no"}">${isPositive ? "Detected" : "Normal"}</span>
    `;
    predictionsList.appendChild(row);
  });

  // Top history tokens
  tokensList.innerHTML = "";
  if (!data.top_history_tokens || data.top_history_tokens.length === 0) {
    tokensList.innerHTML = `<span style="color:var(--muted);font-size:0.85rem">No history tokens</span>`;
  } else {
    data.top_history_tokens.forEach((t, i) => {
      const chip = document.createElement("span");
      chip.className = `token-chip${i < 5 ? " top" : ""}`;
      chip.textContent = t.token;
      chip.title = `attention weight: ${t.weight.toFixed(6)}`;
      tokensList.appendChild(chip);
    });
  }

  emptyState.hidden = true;
  resultsContent.hidden = false;
  resultsPanel.scrollIntoView({ behavior: "smooth", block: "start" });
}
