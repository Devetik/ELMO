# Obtenir tous les fichiers .onnx dans le dossier, triés par date de création
$files = Get-ChildItem -File -Filter "*.onnx" | Sort-Object LastWriteTime

# Initialiser un compteur
$index = 1

# Boucle à travers tous les fichiers .onnx
foreach ($file in $files) {
    # Construire le nouveau nom avec la même extension
    $newName = "$index" + $file.Extension
    Rename-Item $file.FullName -NewName $newName
    $index++
}

Write-Output "Renommage terminé."