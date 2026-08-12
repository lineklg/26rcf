$ErrorActionPreference = 'Stop'

$sourcePath = Join-Path $PSScriptRoot '..\USB_DEVICE\Target\usbd_conf.c'
$source = Get-Content -Raw -LiteralPath $sourcePath

$requiredPatterns = @(
    '__HAL_RCC_GPIOA_CLK_ENABLE\(\)',
    'GPIO_PIN_11\s*\|\s*GPIO_PIN_12',
    'GPIO_MODE_AF_PP',
    'GPIO_AF10_OTG1_HS',
    'HAL_GPIO_Init\(GPIOA,\s*&GPIO_InitStruct\)'
)

foreach ($pattern in $requiredPatterns) {
    if ($source -notmatch $pattern) {
        throw "USB PA11/PA12 配置缺少必需项: $pattern"
    }
}

Write-Output 'USB GPIO configuration test passed.'
