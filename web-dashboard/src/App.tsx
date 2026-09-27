import { useState } from "react";
import { useNTValue, useNTConnection } from "./store/useNetworktables";
import {
  Box,
  Card,
  Container,
  Stack,
  TextField,
  ThemeProvider,
  createTheme,
  CssBaseline
} from "@mui/material";

// 1. Bubbly, Vibrant Theme
// KWARQS 2423 — mallard teal-green & yellow
const KWARQS_GREEN = '#1d7c68';
const KWARQS_YELLOW = '#ffd426';

// 🦆 Checkerboard duck emoji background pattern
const DUCK_PATTERN = `url("data:image/svg+xml,${encodeURIComponent(
  `<svg xmlns='http://www.w3.org/2000/svg' width='100' height='100'>` +
  `<g opacity='0.25' font-size='28' font-family='Apple Color Emoji,Segoe UI Emoji,Noto Color Emoji,sans-serif'>` +
  `<text x='10' y='40'>🦆</text>` +
  `<text x='60' y='90'>🦆</text>` +
  `</g>` +
  `</svg>`
)}")`;


const bubblyTheme = createTheme({
  palette: {
    mode: 'dark',
    primary: { main: KWARQS_GREEN },
    background: { default: '#0a1a0d', paper: 'rgba(15, 35, 18, 0.6)' },
  },
  shape: { borderRadius: 32 },
  typography: {
    fontFamily: '"Nunito", "Inter", sans-serif',
    h4: { fontWeight: 800, letterSpacing: '-0.5px' },
    h6: { fontWeight: 700 },
    subtitle2: { textTransform: 'uppercase', letterSpacing: '1.5px', fontSize: '0.8rem', opacity: 0.9, fontWeight: 700 }
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: {
          background: `${DUCK_PATTERN} repeat, linear-gradient(135deg, #1a1a1a 0%, #212121 100%)`,
          backgroundAttachment: 'fixed',
          height: '100vh',
          overflow: 'hidden',
        },
        '*::-webkit-scrollbar': {
          width: '6px',
        },
        '*::-webkit-scrollbar-track': {
          background: 'transparent',
        },
        '*::-webkit-scrollbar-thumb': {
          background: 'rgba(255,255,255,0.2)',
          borderRadius: '3px',
        },
        '*::-webkit-scrollbar-thumb:hover': {
          background: 'rgba(255,255,255,0.35)',
        },
      }
    },
    MuiCard: {
      styleOverrides: {
        root: {
          backdropFilter: 'blur(16px)',
          border: '1px solid rgba(255,255,255,0.15)',
          boxShadow: '0 8px 32px rgba(0,0,0,0.4)',
        }
      }
    },
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: '50px',
          fontWeight: 800,
          fontSize: '1.1rem',
          textTransform: 'none',
          padding: '14px 28px',
          boxShadow: '0 4px 15px rgba(0,0,0,0.2)'
        }
      }
    },
    MuiOutlinedInput: {
      styleOverrides: { root: { borderRadius: '20px', backgroundColor: 'rgba(0,0,0,0.2)' } }
    },
    MuiMenu: {
      styleOverrides: {
        paper: {
          backgroundColor: '#2a2a2a',
          backdropFilter: 'none',
          border: '1px solid rgba(255,255,255,0.15)',
          boxShadow: '0 8px 32px rgba(0,0,0,0.5)',
        }
      }
    }
  }
});

// Gradients — KWARQS green/yellow team colours
const gradients = {
  glass: `rgba(0, 86, 56, 0.53)`,
  red: 'linear-gradient(135deg, #ff0844 0%, #ffb199 100%)',
  blue: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
  green: `linear-gradient(135deg, ${KWARQS_GREEN} 0%, #56d97a 100%)`,
  warning: `linear-gradient(135deg, ${KWARQS_YELLOW} 0%, #eeff00 100%)`,
  kwarqs: `linear-gradient(135deg, ${KWARQS_GREEN} 0%, ${KWARQS_YELLOW} 100%)`,
};



function App() {
  const { isConnected, address, connect } = useNTConnection();
  const [addressInput, setAddressInput] = useState(address);

  const [, setSomeStringArray] = useNTValue<string[]>("/SmartDashboard/someStringArray");
  const [, setSomeNumberArray] = useNTValue<number[]>("/SmartDashboard/setSomeNumberArray");


  const [isRedAllianceRaw] = useNTValue<boolean>("/FMSInfo/IsRedAlliance", false);
  const isRedAlliance = isRedAllianceRaw ?? false;




  return (
    <ThemeProvider theme={bubblyTheme}>
      <CssBaseline />
      <Container maxWidth={false} sx={{ py: 1, px: { xs: 1, md: 2 } }}>

        {/* Top Navigation / Status Bar */}
        <Stack direction="row" spacing={1.5} sx={{ mb: 1.5, alignItems: 'center' }}>
          <Card sx={{ p: 0.75, px: 2, display: 'flex', alignItems: 'center', gap: 1.5, flex: 1, maxWidth: 420, background: gradients.glass }}>
            <Box sx={{
              width: 12, height: 12, borderRadius: '50%', flexShrink: 0,
              background: isConnected ? gradients.green : gradients.red,
              boxShadow: isConnected ? '0 0 10px #0ba360' : '0 0 10px #ff0844'
            }} />
            <TextField
              size="small"
              fullWidth
              placeholder="NT4 Address (10.TE.AM.2)"
              value={addressInput}
              onChange={(e) => {
                setAddressInput(e.target.value);
                connect(e.target.value);
              }}
              sx={{ '& .MuiOutlinedInput-notchedOutline': { border: 'none' }, '& .MuiInputBase-input': { py: 0.5, fontSize: '0.85rem' } }}
            />
          </Card>
        </Stack>
      </Container>
    </ThemeProvider>
  );
}

export default App;